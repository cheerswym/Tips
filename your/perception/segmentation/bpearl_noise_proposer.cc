#include "onboard/perception/segmentation/bpearl_noise_proposer.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

DEFINE_bool(bpearl_noise_proposer_cvs, false,
            "Enable bpearl noise proposer cvs.");

namespace qcraft::segmentation {

namespace {

constexpr double kMaxBpearlNoiseClusterArea = 0.05;   // m2
constexpr double kMaxBpearlNoiseClusterHeight = 0.3;  // m

Box2d ComputeBpearlNoiseZone(const VehiclePose& pose,
                             const VehicleParamApi& vehicle_params) {
  const auto front_edge_to_center =
      vehicle_params.vehicle_geometry_params().front_edge_to_center();
  const double kBpearlNoiseZoneFront = front_edge_to_center + 4.0;
  const double kBpearlNoiseZoneRear = front_edge_to_center + 1.0;
  constexpr double kBpearlNoiseZoneLateral = 1.0;
  Box2d leaf_zone({pose.x, pose.y}, pose.yaw,
                  kBpearlNoiseZoneFront - kBpearlNoiseZoneRear,
                  kBpearlNoiseZoneLateral * 2);
  leaf_zone.Shift(
      Vec2d((kBpearlNoiseZoneFront + kBpearlNoiseZoneRear) * 0.5, 0.0)
          .FastRotate(pose.yaw));

  if (FLAGS_bpearl_noise_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/bpearl_noise_proposer_cvs");
    canvas.DrawBox({leaf_zone.center(), pose.z}, leaf_zone.heading(),
                   {leaf_zone.length(), leaf_zone.width()}, vis::Color::kBlue);
  }

  return leaf_zone;
}
// Check if cluster only contains left or right bpearl blind lidar points.
bool ContainsSingleBpearlLidarPoints(const ProposedCluster& cluster) {
  int num_left_blind_points = 0;
  int num_right_blind_points = 0;
  int num_total_points = 0;
  for (const auto* obstacle : cluster.obstacles()) {
    for (int i = obstacle->above_ground_points_start_index();
         i < obstacle->points.size(); ++i) {
      const auto& point = obstacle->points[i];
      if (point.lidar_type != static_cast<LidarModel>(LDT_RS_BPEARL)) {
        return false;
      }
      if (point.lidar_id == LDR_FRONT_LEFT_BLIND) {
        ++num_left_blind_points;
      }
      if (point.lidar_id == LDR_FRONT_RIGHT_BLIND) {
        ++num_right_blind_points;
      }
    }
    num_total_points += obstacle->num_points_above_ground;
  }

  return num_left_blind_points == num_total_points ||
         num_right_blind_points == num_total_points;
}

bool IsBpearlNoiseCluster(const ProposedCluster& cluster) {
  if (!ContainsSingleBpearlLidarPoints(cluster)) {
    return false;
  }
  const auto contour = cluster_util::ComputeContourFromClusterPoints(cluster);
  if (contour.area() > kMaxBpearlNoiseClusterArea) {
    return false;
  }
  if (cluster.ComputeHeight() > kMaxBpearlNoiseClusterHeight) {
    return false;
  }
  return true;
}

}  // namespace

ProposedClusters BpearlNoiseProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  const auto& pose = env_info.pose();
  const auto& vehicle_params = env_info.vehicle_params();

  const auto zone_box = ComputeBpearlNoiseZone(pose, vehicle_params);
  ProposedClusters proposed_clusters;
  proposed_clusters.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    if (cluster.HasProperty(PP_NOISE)) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    const auto polygon = cluster_util::ComputeContour(cluster);
    bool is_cluster_in_zone = true;
    for (const auto& point : polygon.points()) {
      if (!zone_box.IsPointIn(point)) {
        is_cluster_in_zone = false;
        break;
      }
    }
    if (!is_cluster_in_zone) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    if (IsBpearlNoiseCluster(cluster)) {
      ProposedCluster proposed_cluster =
          ProposedCluster::InheritFrom(cluster).ConstructBase(cluster);
      proposed_cluster.set_is_proposed(true);
      proposed_cluster.set_property(PP_NOISE);
      proposed_clusters.emplace_back(std::move(proposed_cluster));
    } else {
      proposed_clusters.push_back(cluster);
    }
  }

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
