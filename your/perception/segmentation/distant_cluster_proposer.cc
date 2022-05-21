#include "onboard/perception/segmentation/distant_cluster_proposer.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/math/util.h"
#include "onboard/perception/cluster_util.h"
#include "onboard/perception/semantic_map_util.h"

DEFINE_bool(distant_cluster_proposer_cvs, false,
            "Enable distant cluster proposer cvs.");
namespace qcraft::segmentation {

namespace {

constexpr double kMinDistantRange = 80.0;  // m

bool IsDesiredDistantCluster(const ProposedCluster& cluster,
                             const VehiclePose& pose) {
  if (cluster.HasProperty(PP_NOISE)) return false;
  if (cluster.type() != MT_UNKNOWN && cluster.type() != MT_STATIC_OBJECT) {
    return false;
  }
  const double min_distant_range_sqr = Sqr(kMinDistantRange);
  for (const auto* obstacle : cluster.obstacles()) {
    const double dist_sqr =
        Sqr(obstacle->x - pose.x) + Sqr(obstacle->y - pose.y);
    if (dist_sqr < min_distant_range_sqr) {
      return false;
    }
    const double obs_dir = (obstacle->coord() - Vec2d(pose.x, pose.y)).Angle();
    const double angle_diff = std::abs(NormalizeAngle(obs_dir - pose.yaw));
    if (angle_diff > M_PI_4) {
      return false;
    }
  }
  return true;
}

bool IsSuspectedVehicleCluster(
    const ProposedCluster& cluster,
    const absl::flat_hash_map<LidarId, RangeImage>& range_images) {
  if (range_images.empty()) return false;
  int num_points_on_image = 0;
  int num_points_of_vehicle = 0;
  float average_uncertainty = 0.f;
  for (const auto* obstacle : cluster.obstacles()) {
    for (const auto& point : obstacle->points) {
      const auto* range_image = FindOrNull(range_images, point.lidar_id);
      SegmentationType type = ST_DONTCARE;
      float uncertainty = 0.f;
      if (range_image) {
        const auto& semantic_image = range_image->semantic_image();
        const auto [row, col] = range_image->ImagePosAt(
            point.scan_or_point_index, point.beam_index);
        const auto vec2 = semantic_image.at<cv::Vec2b>(row, col);
        type = static_cast<SegmentationType>(vec2[0]);
        uncertainty = vec2[1] * (1.f / 255.f);
      }
      if (type == ST_DONTCARE) continue;
      ++num_points_on_image;
      if (type == ST_CAR) {
        ++num_points_of_vehicle;
        average_uncertainty += uncertainty;
      }
    }
  }
  constexpr double kMinOnImagePointsRatio = 0.5;
  constexpr double kMinVehiclePointRatio = 0.5;
  if (num_points_on_image < cluster.NumPoints() * kMinOnImagePointsRatio) {
    return false;
  }
  if (num_points_of_vehicle < num_points_on_image * kMinVehiclePointRatio) {
    return false;
  }
  average_uncertainty /= num_points_of_vehicle;
  constexpr double kMinAverageUncertainty = 0.5;
  if (average_uncertainty < kMinAverageUncertainty) {
    return false;
  }
  return true;
}

bool IsTargetVehicleCluster(
    const ProposedCluster& cluster, const double lane_heading,
    const absl::flat_hash_map<LidarId, RangeImage>& range_images) {
  const auto contour = cluster_util::ComputeContour(cluster);
  const auto box = contour.BoundingBoxWithHeading(lane_heading);
  constexpr double kMinVehicleClusterWidth = 1.4;   // m
  constexpr double kMaxVehicleClusterWidth = 4.5;   // m
  constexpr double kMinVehicleClusterArea = 0.3;    // m^2
  constexpr double kMaxVehicleClusterArea = 20.0;   // m^2
  constexpr double kMinVehicleClusterHeight = 0.7;  // m
  if (box.width() < kMinVehicleClusterWidth ||
      box.width() > kMaxVehicleClusterWidth ||
      contour.area() < kMinVehicleClusterArea ||
      contour.area() > kMaxVehicleClusterArea ||
      cluster.ComputeHeight() < kMinVehicleClusterHeight) {
    return false;
  }
  return IsSuspectedVehicleCluster(cluster, range_images);
}
// TODO(dong): Impl later.
bool IsOverSegmentedCluster(const ProposedCluster& cluster) { return true; }

void MaybeRenderClusterRoiBox(const Box2d& box) {
  if (FLAGS_distant_cluster_proposer_cvs) {
    vis::Canvas& canvas = vantage_client_man::GetCanvas(
        "perception/distant_cluster_proposer_cvs");
    canvas.DrawBox(Vec3d(box.center(), 0.0), box.heading(),
                   {box.length(), box.width()}, vis::Color::kIndigo);
  }
}

Box2d ComputeVehicleClusterRoi(const Polygon2d& contour,
                               const double lane_heading) {
  Box2d box = contour.BoundingBoxWithHeading(lane_heading);
  constexpr double kMaxSmallVehicleWidth = 2.5;  // m
  double roi_length = 0.0;                       // m
  if (box.width() < kMaxSmallVehicleWidth) {
    roi_length = std::clamp((box.width() - 1.5) * 0.5 + 4.0, 4.0, 5.0);
  } else {
    roi_length = std::clamp((box.width() - 2.0) * 10.0, 6.0, 12.0);
    roi_length = std::max(roi_length, box.length() + 2.0);
  }
  box.Shift(
      Vec2d((roi_length - box.length()) * 0.5, 0.0).FastRotate(lane_heading));
  box.LongitudinalExtend(roi_length - box.length());
  box.LateralExtend(0.8);

  MaybeRenderClusterRoiBox(box);

  return box;
}

ProposedCluster MergeTwoProposedClusters(const ProposedCluster& lhs,
                                         const ProposedCluster& rhs) {
  ObstaclePtrs obstacles;
  obstacles.reserve(lhs.obstacles().size() + rhs.obstacles().size());
  obstacles.insert(obstacles.end(), lhs.obstacles().begin(),
                   lhs.obstacles().end());
  obstacles.insert(obstacles.end(), rhs.obstacles().begin(),
                   rhs.obstacles().end());
  ProposedCluster proposed_cluster = ProposedCluster::InheritFrom({lhs, rhs})
                                         .ConstructBase(std::move(obstacles));
  proposed_cluster.set_is_proposed(true);

  return proposed_cluster;
}

ProposedClusters ProposeDistantClusters(const ProposerEnvInfo& env_info,
                                        ProposedClusters distant_clusters) {
  const auto& pose = env_info.pose();
  const auto& range_images = env_info.context().range_images;
  const auto compare_range = [&](const auto& lhs, const auto& rhs) {
    const auto* lhs_obs =
        *std::min_element(lhs.obstacles().begin(), lhs.obstacles().end(),
                          [&](const auto* l, const auto* r) {
                            return Sqr(l->x - pose.x) + Sqr(l->y - pose.y) <
                                   Sqr(r->x - pose.x) + Sqr(r->y - pose.y);
                          });
    const auto* rhs_obs =
        *std::min_element(rhs.obstacles().begin(), rhs.obstacles().end(),
                          [&](const auto* l, const auto* r) {
                            return Sqr(l->x - pose.x) + Sqr(l->y - pose.y) <
                                   Sqr(r->x - pose.x) + Sqr(r->y - pose.y);
                          });
    return std::make_pair(Sqr(lhs_obs->x - pose.x) + Sqr(lhs_obs->y - pose.y),
                          lhs_obs->coord()) <
           std::make_pair(Sqr(rhs_obs->x - pose.x) + Sqr(rhs_obs->y - pose.y),
                          rhs_obs->coord());
  };
  std::sort(distant_clusters.begin(), distant_clusters.end(), compare_range);
  const auto& semantic_map_manager = env_info.semantic_map_manager();
  std::vector<Polygon2d> contours(distant_clusters.size());
  for (int i = 0; i < distant_clusters.size(); ++i) {
    contours[i] = cluster_util::ComputeContour(distant_clusters[i]);
  }
  std::vector<bool> visited(distant_clusters.size(), false);
  ProposedClusters merged_clusters;
  for (int i = 0; i < distant_clusters.size(); ++i) {
    const auto& cluster = distant_clusters[i];
    const auto& contour = contours[i];
    if (visited[i]) continue;
    const auto lane_heading =
        perception_semantic_map_util::ComputeLaneHeadingAtPos(
            semantic_map_manager, contour.centroid());
    if (!lane_heading) continue;
    if (!IsTargetVehicleCluster(cluster, *lane_heading, range_images)) {
      continue;
    }
    // Merge over segmented clusters.
    auto current_cluster = cluster;
    std::optional<double> init_box_width;
    do {
      const auto current_contour =
          cluster_util::ComputeContour(current_cluster);
      const Box2d roi_box =
          ComputeVehicleClusterRoi(current_contour, *lane_heading);
      if (!init_box_width) {
        init_box_width = roi_box.width();
      }
      if (roi_box.width() > *init_box_width * 1.5) break;
      const Polygon2d roi(roi_box);
      bool have_over_segmented_cluster = false;
      for (int j = 0; j < distant_clusters.size(); ++j) {
        if (j == i || visited[j]) continue;
        if (!IsOverSegmentedCluster(distant_clusters[j])) continue;
        const auto& ctr = contours[j];
        Polygon2d intersection;
        if (!roi.ComputeOverlap(ctr, &intersection)) continue;
        constexpr double kMinOverlapRatioForMerge = 0.75;
        if (intersection.area() > ctr.area() * kMinOverlapRatioForMerge) {
          have_over_segmented_cluster = true;
          visited[i] = true;
          visited[j] = true;
          current_cluster =
              MergeTwoProposedClusters(current_cluster, distant_clusters[j]);
        }
      }
      if (!have_over_segmented_cluster) break;
    } while (true);

    if (visited[i]) {
      merged_clusters.emplace_back(std::move(current_cluster));
    }
  }

  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(distant_clusters.size());
  for (int i = 0; i < distant_clusters.size(); ++i) {
    if (visited[i]) continue;
    proposed_clusters.emplace_back(distant_clusters[i]);
  }

  proposed_clusters.insert(proposed_clusters.end(),
                           std::make_move_iterator(merged_clusters.begin()),
                           std::make_move_iterator(merged_clusters.end()));

  return proposed_clusters;
}

}  // namespace

ProposedClusters DistantClusterProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  const auto& pose = env_info.pose();
  //
  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(clusters.size());
  ProposedClusters distant_clusters;
  distant_clusters.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    if (IsDesiredDistantCluster(cluster, pose)) {
      distant_clusters.emplace_back(cluster);
    } else {
      proposed_clusters.emplace_back(cluster);
    }
  }

  auto proposed_distant_clusters =
      ProposeDistantClusters(env_info, std::move(distant_clusters));

  proposed_clusters.insert(
      proposed_clusters.end(),
      std::make_move_iterator(proposed_distant_clusters.begin()),
      std::make_move_iterator(proposed_distant_clusters.end()));

  QCHECK_EQ(CalculateTotalNumObstacles(clusters),
            CalculateTotalNumObstacles(proposed_clusters));

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
