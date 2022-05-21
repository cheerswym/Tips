#include "onboard/perception/segmentation/semantic_map_proposer.h"

#include <algorithm>
#include <array>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/perception/semantic_map_util.h"

DEFINE_bool(semantic_map_proposer_cvs, false,
            "Whether to render semantic map proposer cvs.");

namespace qcraft::segmentation {

using namespace std::placeholders;  // NOLINT

namespace {

constexpr float kVegetationClusterMinRatio = 0.8;
constexpr float kBarrierClusterMinRatio = 0.8;
constexpr double kMinVegetationObstacleRatio = 0.10;
constexpr double kMinBarrierObstacleRatio = 0.10;

bool ExceedsBarrierWidth(const ProposedCluster& cluster) {
  const auto contour = cluster_util::ComputeContour(cluster);
  const auto min_area_box = contour.MinAreaBoundingBox();
  constexpr float kMaxBarrierWidth = 3 * Obstacle::kDiameter;
  return std::min(min_area_box.width(), min_area_box.length()) >
         kMaxBarrierWidth;
}

// Returns if the cluster is over-sized. Cluster is over-sized when a cluster is
// too hollow around edges in terms of the whole cluster contour and occupied
// obstacles inside, i.e. not enough obstacles to support a certain edge of the
// cluster. Specifically, we tell if it's oversized by check if the edge centers
// of cluster is too far away from obstacles.
bool IsOversizedCluster(const ProposedCluster& cluster,
                        const SemanticMapManager& semantic_map_manager) {
  const auto cluster_contour = cluster_util::ComputeContour(cluster);
  const auto cluster_edges = cluster_contour.line_segments();

  for (const auto& edge : cluster_edges) {
    // If edge is too short, no need to process it.
    if (edge.length() <= 10 * Obstacle::kDiameter) {
      continue;
    }
    const auto edge_center = edge.center();
    mapping::ElementId lane_id;
    double fraction;
    Vec2d proj_point;
    double min_dist;
    semantic_map_manager.GetNearestLaneProjectionAtLevel(
        semantic_map_manager.GetLevel(), edge_center, &lane_id, &fraction,
        &proj_point, &min_dist);

    // If edge is too far away from lane, no need to process it since it won't
    // affect driving.
    constexpr float kLaneWidth = 3.7;  // m
    if (min_dist >= kLaneWidth / 2) {
      continue;
    }

    // If edge is not facing the lane, no need to process it.
    const Vec2d edge_facing_out_normal = -edge.unit_direction().Perp();
    const Vec2d edge_center_to_lane = proj_point - edge_center;
    if (edge_facing_out_normal.Dot(edge_center_to_lane) < 0 &&
        !cluster_contour.IsPointIn(proj_point)) {
      continue;
    }

    if (FLAGS_semantic_map_proposer_cvs) {
      vis::Canvas& canvas =
          vantage_client_man::GetCanvas("perception/semantic_map_proposer");
      canvas.DrawLine({edge_center, 0.0}, {proj_point, 0.0}, vis::Color::kBlue);
      canvas.DrawLine({edge_center, 0.0},
                      {edge_center + edge_facing_out_normal, 0.0},
                      vis::Color::kRed);
    }

    float edge_distance_sqr = std::numeric_limits<float>::max();
    for (const auto& obstacle : cluster.obstacles()) {
      edge_distance_sqr = std::min(
          edge_distance_sqr,
          static_cast<float>(obstacle->coord().DistanceSquareTo(edge_center)));
    }
    // If the center of edges is kMinDistanceSqrToRealObstacle squared distance
    // away from any real obstacles, it's a oversized cluster.
    constexpr float kMinDistanceSqrToRealObstacle =
        Sqr(2.5 * Obstacle::kDiameter);
    if (edge_distance_sqr > kMinDistanceSqrToRealObstacle) {
      return true;
    }
  }
  return false;
}

// Only segment clusters if these are vegetation or barrier.
// Segment vegetation if oversized and do so on barrier if oversized and also
// exceed barrier width.
bool ShouldSegmentClusterBySemanticMap(
    const ProposedCluster& cluster,
    const SemanticMapManager& semantic_map_manager, const bool is_veg,
    const bool is_barrier) {
  if (is_veg) {
    return IsOversizedCluster(cluster, semantic_map_manager);
  } else if (is_barrier) {
    return IsOversizedCluster(cluster, semantic_map_manager) &&
           ExceedsBarrierWidth(cluster);
  }
  return false;
}

std::array<int, 3> CountsVegetationBarrierAndUnignoredObstacleNumber(
    const Cluster& cluster) {
  // Count number of obstacles in zone, and classifiy based on obstacle ratio.
  int num_vegetation_obstacles = 0;
  int num_barrier_obstacles = 0;
  int num_unignored_obstacles = 0;
  for (const auto* obstacle : cluster.obstacles()) {
    if (obstacle->type == ObstacleProto::VEGETATION) {
      num_vegetation_obstacles += 1;
    } else if (obstacle->type == ObstacleProto::BARRIER) {
      num_barrier_obstacles += 1;
    }
    if (obstacle->type != ObstacleProto::IGNORED) {
      num_unignored_obstacles += 1;
    }
  }
  return {num_vegetation_obstacles, num_barrier_obstacles,
          num_unignored_obstacles};
}

std::pair<bool, bool> IsSuspectedVegetationOrBarrierCluster(
    const Cluster& cluster) {
  const auto [num_vegetation_obstacles, num_barrier_obstacles,
              num_unignored_obstacles] =
      CountsVegetationBarrierAndUnignoredObstacleNumber(cluster);
  if (num_vegetation_obstacles == 0 && num_barrier_obstacles == 0) {
    return {false, false};
  }

  if (static_cast<float>(num_vegetation_obstacles) / num_unignored_obstacles <
          kMinVegetationObstacleRatio &&
      static_cast<float>(num_barrier_obstacles) / num_unignored_obstacles <
          kMinBarrierObstacleRatio) {
    return {false, false};
  }

  return {num_vegetation_obstacles >= num_barrier_obstacles,
          !(num_vegetation_obstacles >= num_barrier_obstacles)};
}

std::pair<bool, bool> IsVegetationOrBarrierCluster(const Cluster& cluster) {
  // Count number of obstacles in zone, and classifiy based on obstacle ratio.
  const auto [num_vegetation_obstacles, num_barrier_obstacles,
              num_unignored_obstacles] =
      CountsVegetationBarrierAndUnignoredObstacleNumber(cluster);
  if (num_unignored_obstacles == 0) return {false, false};
  const float veg_obstacle_ratio =
      num_vegetation_obstacles * 1.0f / num_unignored_obstacles;
  const float barrier_obstacle_ratio =
      num_barrier_obstacles * 1.0f / num_unignored_obstacles;
  const bool is_veg = veg_obstacle_ratio >= kVegetationClusterMinRatio;
  const bool is_barrier = barrier_obstacle_ratio >= kBarrierClusterMinRatio;

  return {is_veg, is_barrier};
}

void AlterOutReachingTreeBranches(const ProposerEnvInfo& env_info,
                                  ProposedClusters* clusters) {
  const auto vegetation_zones =
      env_info.GetNearPerceptionZones(mapping::PerceptionZoneProto::VEGETATION);

  vis::Canvas* canvas = nullptr;
  if (FLAGS_semantic_map_proposer_cvs) {
    canvas = &vantage_client_man::GetCanvas("perception/semantic_map_proposer");
  }

  for (auto& cluster : *clusters) {
    // Do not overwrite FEN/LLN detection result nor semantic map proposed.
    if (cluster.type_source() == MTS_LL_NET ||
        cluster.type_source() == MTS_FIERY_EYE_NET ||
        cluster.type_source() == MTS_SEMANTIC_MAP_ZONE ||
        cluster.HasProperty(PP_NOISE)) {
      continue;
    }

    const auto cluster_contour = cluster_util::ComputeContour(cluster);

    // Should skip if the cluster is too far away from veg zone or curb.
    constexpr double kMaxOutReachingTreeBranchesInnerDist = 0.2;   // m.
    constexpr double kMaxOutReachingTreeBranchesOutterDist = 1.2;  // m.
    double min_dist_to_veg = std::numeric_limits<double>::max();
    double max_dist_to_veg = std::numeric_limits<double>::lowest();
    int closest_zone_index = -1;
    for (int i = 0; i < vegetation_zones.size(); ++i) {
      const auto& veg_zone = vegetation_zones[i];
      min_dist_to_veg =
          std::min(min_dist_to_veg, veg_zone.DistanceTo(cluster_contour));
      closest_zone_index = i;
    }
    if (closest_zone_index != -1) {
      const auto& veg_zone = vegetation_zones[closest_zone_index];
      for (const auto& point : cluster_contour.points()) {
        max_dist_to_veg = std::max(max_dist_to_veg, veg_zone.DistanceTo(point));
      }
    } else {
      max_dist_to_veg = 1e5;
    }
    const bool near_veg_zone_suspect_region =
        min_dist_to_veg < kMaxOutReachingTreeBranchesInnerDist &&
        max_dist_to_veg < kMaxOutReachingTreeBranchesOutterDist;

    float min_dist_to_curb = std::numeric_limits<float>::max();
    float max_dist_to_curb = std::numeric_limits<float>::lowest();
    QCHECK_GT(cluster.obstacles().size(), 0);
    for (const auto* obstacle : cluster.obstacles()) {
      min_dist_to_curb =
          std::min(std::fabs(obstacle->dist_to_curb), min_dist_to_curb);
      max_dist_to_curb =
          std::max(std::fabs(obstacle->dist_to_curb), max_dist_to_curb);
    }
    const bool near_curb_suspect_region =
        min_dist_to_curb < kMaxOutReachingTreeBranchesInnerDist &&
        max_dist_to_curb < kMaxOutReachingTreeBranchesOutterDist;
    if (!near_veg_zone_suspect_region && !near_curb_suspect_region) {
      if (FLAGS_semantic_map_proposer_cvs) {
        canvas->DrawText(
            absl::StrFormat(
                "Cluster too far: veg %.2f -> %.2f, curb %.2f -> %.2f",
                min_dist_to_veg, max_dist_to_veg, min_dist_to_curb,
                max_dist_to_curb),
            {cluster_contour.points()[0], 0.0}, 0.0, 0.1,
            vis::Color::kLightGreen);
      }
      continue;
    }

    // Should skip if the mean clearance is too low.
    const double kMinTreeBranchesAverageClearance = 1.6;  // m.
    double averaged_clearance = 0.;
    for (const auto* obstacle : cluster.obstacles()) {
      averaged_clearance += obstacle->clearance;
    }
    averaged_clearance *= (1.0 / cluster.obstacles().size());
    if (averaged_clearance < kMinTreeBranchesAverageClearance) {
      if (FLAGS_semantic_map_proposer_cvs) {
        canvas->DrawText(
            absl::StrFormat("Cluster too low: %.2f", averaged_clearance),
            {cluster_contour.points()[0], 0.0}, 0.0, 0.1,
            vis::Color::kLightGreen);
      }
      continue;
    }

    // Should skip if the mean intensity is too high.
    // Use different intensity threshold for veg/curb since we believe for now
    // that cluster inside veg zone is more likely to be veg than curb.
    constexpr double kMaxAveragedIntensityForVegZone = 120;
    constexpr double kMaxAveragedIntensityForCurb = 50;
    int num_points = 0;
    double averaged_intensity = 0.;
    for (const auto* obstacle : cluster.obstacles()) {
      for (const auto& point : obstacle->points) {
        ++num_points;
        averaged_intensity += point.intensity;
      }
    }
    averaged_intensity /= num_points;
    // Use veg zone threshold first if it's near veg zone and use curb threshold
    // otherwise.
    const double intensity_threshold = near_veg_zone_suspect_region
                                           ? kMaxAveragedIntensityForVegZone
                                           : kMaxAveragedIntensityForCurb;
    if (averaged_intensity > intensity_threshold) {
      if (FLAGS_semantic_map_proposer_cvs) {
        canvas->DrawText(absl::StrFormat("Cluster's intensity too high: %.2f",
                                         averaged_intensity),
                         {cluster_contour.points()[0], 0.0}, 0.0, 0.1,
                         vis::Color::kLightGreen);
      }
      continue;
    }
    cluster.set_type(MT_VEGETATION);
    cluster.set_type_source(MTS_SEMANTIC_MAP_ZONE);
    cluster.set_is_proposed(true);
  }
}

ProposedClusters SegmentSuspectedVegetationAndBarrierClusters(
    const ProposedClusters& clusters) {
  ProposedClusters proposed_clusters;
  for (const auto& cluster : clusters) {
    if (cluster.type() == MT_VEGETATION || cluster.type() == MT_BARRIER) {
      proposed_clusters.emplace_back(cluster);
      continue;
    }
    const auto [is_suspected_veg, is_suspected_barrier] =
        IsSuspectedVegetationOrBarrierCluster(cluster);
    if (!is_suspected_veg && !is_suspected_barrier) {
      proposed_clusters.emplace_back(cluster);
      continue;
    }
    QCHECK(!(is_suspected_veg && is_suspected_barrier));
    ProposedCluster sorted_cluster = cluster;
    std::stable_sort(sorted_cluster.mutable_obstacles()->begin(),
                     sorted_cluster.mutable_obstacles()->end(),
                     [&](const Obstacle* a, const Obstacle* b) {
                       return a->type == ObstacleProto::VEGETATION &&
                              b->type != ObstacleProto::VEGETATION;
                     });
    const obstacle_util::NeighborRange neighbor_range(
        [](const Obstacle& obstacle) {
          return obstacle.type == ObstacleProto::VEGETATION ? 3 : 1;
        });
    const auto connected_condition = [](const Obstacle& init,
                                        const Obstacle& seed,
                                        const Obstacle& neighbor) {
      return init.type == ObstacleProto::VEGETATION
                 ? (seed.type == ObstacleProto::VEGETATION &&
                    neighbor.type == ObstacleProto::STATIC) ||
                       (neighbor.type == ObstacleProto::VEGETATION)
                 : (seed.type == neighbor.type);
    };
    auto segmented_clusters = SegmentAndProposeClusterWithConnectedComponents(
        cluster, neighbor_range, connected_condition);
    for (auto& segmented_cluster : segmented_clusters) {
      const auto [is_veg, is_barrier] =
          IsVegetationOrBarrierCluster(segmented_cluster);
      if (is_veg) {
        segmented_cluster.set_type(MT_VEGETATION);
        segmented_cluster.set_type_source(MTS_SEMANTIC_MAP_ZONE);
      }
      if (is_barrier) {
        segmented_cluster.set_type(MT_BARRIER);
        segmented_cluster.set_type_source(MTS_SEMANTIC_MAP_ZONE);
      }
      proposed_clusters.emplace_back(std::move(segmented_cluster));
    }
  }

  return proposed_clusters;
}

ProposedClusters ProposeClustersBySemanticMap(const ProposedClusters& clusters,
                                              const ProposerEnvInfo& env_info) {
  const auto& semantic_map_manager = env_info.semantic_map_manager();

  ProposedClusters proposed_clusters;
  for (const auto& cluster : clusters) {
    // Do not overwrite FEN/LLN detection result for semantic map zone.
    if (cluster.type_source() == MTS_LL_NET ||
        cluster.type_source() == MTS_FIERY_EYE_NET ||
        cluster.HasProperty(PP_NOISE)) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    const auto [is_suspected_veg, is_suspected_barrier] =
        IsSuspectedVegetationOrBarrierCluster(cluster);
    if (!is_suspected_veg && !is_suspected_barrier) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    // Tackle the cases where undersegmented clusters generates over-sized
    // clusters.

    auto segmented_clusters = SegmentAndProposeClusterIf(
        cluster, std::bind(ShouldSegmentClusterBySemanticMap, _1,
                           std::cref(semantic_map_manager), is_suspected_veg,
                           is_suspected_barrier));
    for (auto& segmented_cluster : segmented_clusters) {
      const auto [is_veg, is_barrier] =
          IsVegetationOrBarrierCluster(segmented_cluster);
      if (is_veg) {
        segmented_cluster.set_type(MT_VEGETATION);
        segmented_cluster.set_type_source(MTS_SEMANTIC_MAP_ZONE);
      }
      if (is_barrier) {
        segmented_cluster.set_type(MT_BARRIER);
        segmented_cluster.set_type_source(MTS_SEMANTIC_MAP_ZONE);
      }
      segmented_cluster.set_is_proposed(true);
    }

    segmented_clusters =
        SegmentSuspectedVegetationAndBarrierClusters(segmented_clusters);

    proposed_clusters.insert(
        proposed_clusters.end(),
        std::make_move_iterator(segmented_clusters.begin()),
        std::make_move_iterator(segmented_clusters.end()));
  }

  // BANDAID(yu, dong): Mark the following cases as vegetation to tackle
  // out-reaching tree branches.
  AlterOutReachingTreeBranches(env_info, &proposed_clusters);

  return proposed_clusters;
}

ProposedClusters ProposeForXXXXLClusters(
    const ProposedClusters& clusters,
    const SemanticMapManager& semantic_map_manager) {
  ProposedClusters proposed_clusters;
  for (const auto& cluster : clusters) {
    // Do not overwrite FEN/LLN detection result for semantic map zone.
    // Do not overwrite if it's trimed (with kNoise property).
    if (cluster.type_source() == MTS_LL_NET ||
        cluster.type_source() == MTS_FIERY_EYE_NET ||
        cluster.HasProperty(PP_NOISE)) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    const auto cluster_contour = cluster_util::ComputeContour(cluster);
    const auto min_area_bb = cluster_contour.MinAreaBoundingBox();

    constexpr double kMaxReasonableClusterArea = 100.;  // m^2
    // The largest vehicle observed is 26m.
    constexpr double kMaxReasonbaleClusterLength = 30.;  // m
    if (cluster_contour.area() < kMaxReasonableClusterArea &&
        std::max(min_area_bb.length(), min_area_bb.width()) <
            kMaxReasonbaleClusterLength) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    const auto segmented_clusters = SegmentAndProposeClusterIf(
        cluster,
        std::bind(IsOversizedCluster, _1, std::cref(semantic_map_manager)));
    proposed_clusters.insert(proposed_clusters.end(),
                             segmented_clusters.begin(),
                             segmented_clusters.end());
  }
  return proposed_clusters;
}

}  // namespace

ProposedClusters SemanticMapProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  const auto proposed_clusters_after_semantic_map =
      ProposeClustersBySemanticMap(clusters, env_info);
  // TODO(dongchen, yu): Separate the following logic out as a separate proposer
  // to both classify and segment for large clusters.
  return ProposeForXXXXLClusters(proposed_clusters_after_semantic_map,
                                 env_info.semantic_map_manager());
}

}  // namespace qcraft::segmentation
