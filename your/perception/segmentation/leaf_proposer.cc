#include "onboard/perception/segmentation/leaf_proposer.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/geometry/affine_transformation.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/obstacle_util.h"
#include "onboard/perception/segmentation/proposer_util.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(leaf_proposer_cvs, false, "Render leaf proposer results");

namespace qcraft::segmentation {

namespace {
// Suspected leaf cluster param
constexpr int kMaxSuspectedLeafClusterNumObstacles = 6;
constexpr double kMaxSuspectedLeafClusterArea = 0.15;  // radius 0.22m
constexpr float kMaxSuspectedLeafClusterDeltaZ = 0.3;  // m
// Leaf cluster param
constexpr int kMaxLeafClusterNumObstacles = 4;
constexpr double kMaxLeafClusterArea = 0.05;   // radius 0.13m
constexpr float kMaxLeafClusterDeltaZ = 0.25;  // m

constexpr float kMinLeafClusterClearance = 0.5;  // m

// Returns if cluster is road agent or cone. We don't filter these clusters.
bool IsClusterLeafFilterable(const Cluster& cluster) {
  return !(cluster.type() == MT_VEHICLE || cluster.type() == MT_PEDESTRIAN ||
           cluster.type() == MT_CYCLIST || cluster.type() == MT_MOTORCYCLIST ||
           cluster.type() == MT_CONE);
}

Box2d ComputeLeafZone(const VehiclePose& pose,
                      const VehicleParamApi& vehicle_params) {
  QCHECK(vehicle_params.has_vehicle_geometry_params());
  constexpr double kLeafZoneFront = 40.0;
  constexpr double kLeafZoneLateral = 6.0;
  const auto front_edge_to_center =
      vehicle_params.vehicle_geometry_params().front_edge_to_center();
  const double kLeafZoneRear = front_edge_to_center + 0.2;
  Box2d leaf_zone({pose.x, pose.y}, pose.yaw, kLeafZoneFront - kLeafZoneRear,
                  kLeafZoneLateral * 2);
  leaf_zone.Shift(
      Vec2d((kLeafZoneFront + kLeafZoneRear) * 0.5, 0.0).FastRotate(pose.yaw));

  if (FLAGS_leaf_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/leaf_proposer_cvs");
    canvas.DrawBox({leaf_zone.center(), pose.z}, leaf_zone.heading(),
                   {leaf_zone.length(), leaf_zone.width()}, vis::Color::kGreen);
  }

  return leaf_zone;
}

std::pair<float, float> GetClusterDeltaZAndAverageClearance(
    const Cluster& cluster, const float av_height) {
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  float avg_clearance = 0.0;
  bool has_above_ground_points = false;
  for (const auto* obstacle : cluster.obstacles()) {
    for (const auto& p : obstacle->points) {
      if (obstacle_util::IsAboveGroundObstaclePoint(*obstacle, p)) {
        min_z = std::min(min_z, p.z);
        max_z = std::max(max_z, p.z);
        has_above_ground_points = true;
      }
    }
    avg_clearance += obstacle->clearance;
  }
  QCHECK(has_above_ground_points);
  avg_clearance /= cluster.NumObstacles();
  return {max_z - min_z, avg_clearance};
}

bool AreSimilarLeafClusters(const Cluster& cluster_1,
                            const LeafClusterInfo& cluster_info,
                            const float av_height) {
  const auto& contour_1 =
      cluster_util::ComputeContourFromClusterPoints(cluster_1);
  const auto& contour_2 = cluster_info.contour;

  constexpr double kMaxCentroidDistance = 0.1;
  if ((contour_1.centroid() - contour_2.centroid()).squaredNorm() >
      Sqr(kMaxCentroidDistance)) {
    return false;
  }
  constexpr double kMaxAreaDifference = 0.02;
  if (std::abs(contour_1.area() - contour_2.area()) > kMaxAreaDifference) {
    return false;
  }
  const auto [delta_z_1, avg_clearance_1] =
      GetClusterDeltaZAndAverageClearance(cluster_1, av_height);
  constexpr float kMaxDeltaZDifference = 0.1;
  constexpr float kMaxAverageClearanceDifference = 0.1;
  if (std::abs(delta_z_1 - cluster_info.delta_z) > kMaxDeltaZDifference ||
      std::abs(avg_clearance_1 - cluster_info.avg_clearance) >
          kMaxAverageClearanceDifference) {
    return false;
  }

  return true;
}

bool SegmentAndCheckSuspectedLeafCluster(const ProposedCluster& cluster,
                                         const float av_height) {
  const auto segmented_clusters =
      SegmentAndProposeClusterWithConnectedComponents(cluster, 4);
  for (const auto& segmented_cluster : segmented_clusters) {
    if (segmented_cluster.NumObstacles() > kMaxLeafClusterNumObstacles) {
      return false;
    }
    QCHECK(!segmented_cluster.obstacles().empty());
    const auto& contour =
        cluster_util::ComputeContourFromClusterPoints(segmented_cluster);
    if (contour.area() > kMaxLeafClusterArea) {
      return false;
    }
    const auto [delta_z, avg_clearance] =
        GetClusterDeltaZAndAverageClearance(segmented_cluster, av_height);
    if (delta_z > kMaxLeafClusterDeltaZ ||
        avg_clearance < kMinLeafClusterClearance) {
      return false;
    }
  }
  return true;
}

std::optional<LeafClusterInfo> ComputeLeafClusterInfo(
    const ProposedCluster& cluster, const float av_height,
    const Box2d& leaf_zone) {
  if (!IsClusterLeafFilterable(cluster)) {
    return std::nullopt;
  }
  if (cluster.NumObstacles() > kMaxSuspectedLeafClusterNumObstacles) {
    return std::nullopt;
  }
  for (const auto* obstacle : cluster.obstacles()) {
    if (!leaf_zone.IsPointIn(obstacle->coord())) {
      return std::nullopt;
    }
  }

  QCHECK(!cluster.obstacles().empty());
  const auto& contour = cluster_util::ComputeContourFromClusterPoints(cluster);
  if (contour.area() > kMaxSuspectedLeafClusterArea) {
    return std::nullopt;
  }

  const auto [delta_z, avg_clearance] =
      GetClusterDeltaZAndAverageClearance(cluster, av_height);
  const bool is_leaf = delta_z < kMaxSuspectedLeafClusterDeltaZ &&
                       avg_clearance > kMinLeafClusterClearance;
  if (!is_leaf) {
    return std::nullopt;
  }

  if (!SegmentAndCheckSuspectedLeafCluster(cluster, av_height)) {
    return std::nullopt;
  }

  return std::make_optional<LeafClusterInfo>({contour, delta_z, avg_clearance});
}

}  // namespace

ProposedClusters LeafProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  const auto& pose = env_info.pose();
  const auto& av_height = env_info.av_height();
  const auto& vehicle_params = env_info.vehicle_params();

  const auto leaf_zone = ComputeLeafZone(pose, vehicle_params);

  if (prev_leaf_clusters_info_.empty()) {
    for (const auto& cluster : clusters) {
      const auto leaf_cluster_info =
          ComputeLeafClusterInfo(cluster, av_height, leaf_zone);
      if (leaf_cluster_info) {
        prev_leaf_clusters_info_.emplace_back(*leaf_cluster_info);
      }
    }
    return clusters;
  }

  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(clusters.size());
  std::vector<LeafClusterInfo> curr_leaf_clusters_info;
  for (const auto& cluster : clusters) {
    if (cluster.HasProperty(PP_NOISE)) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    bool has_similar_prev_cluster = false;
    const auto leaf_cluster_info =
        ComputeLeafClusterInfo(cluster, av_height, leaf_zone);
    if (leaf_cluster_info) {
      curr_leaf_clusters_info.emplace_back(*leaf_cluster_info);

      for (const auto& prev_cluster_info : prev_leaf_clusters_info_) {
        if (AreSimilarLeafClusters(cluster, prev_cluster_info, av_height)) {
          has_similar_prev_cluster = true;
          break;
        }
      }

      if (FLAGS_leaf_proposer_cvs) {
        vis::Canvas& canvas =
            vantage_client_man::GetCanvas("perception/leaf_proposer_cvs");
        const std::string debug_string = absl::StrFormat(
            "maybe leaf? %s, has_similar_prev_cluster? %s, \n"
            "num obstacles: %d, num points: %d, area: %.3f, delta_z: %.3f, \n"
            "avg_clearance: %.3f",
            leaf_cluster_info ? "YES" : "NO",
            has_similar_prev_cluster ? "YES" : "NO", cluster.NumObstacles(),
            cluster.NumPoints(), leaf_cluster_info->contour.area(),
            leaf_cluster_info->delta_z, leaf_cluster_info->avg_clearance);
        const auto coord = cluster.obstacles()[0]->coord();
        canvas.DrawText(debug_string,
                        Vec3d(coord.x(), coord.y() - 0.2, pose.z + 0.1), 0,
                        0.05, vis::Color::kWhite);
      }
    }
    if (leaf_cluster_info && !has_similar_prev_cluster) {
      ProposedCluster proposed_cluster =
          ProposedCluster::InheritFrom(cluster).ConstructBase(cluster);
      proposed_cluster.set_is_proposed(true);
      proposed_cluster.set_property(PP_NOISE);
      proposed_clusters.emplace_back(std::move(proposed_cluster));
    } else {
      proposed_clusters.emplace_back(cluster);
    }
  }
  prev_leaf_clusters_info_.swap(curr_leaf_clusters_info);

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
