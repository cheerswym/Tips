#include "onboard/perception/segmentation/ouster_noise_proposer.h"

#include <cmath>
#include <utility>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

DEFINE_bool(ouster_noise_proposer_cvs, false,
            "Enable bpearl noise proposer cvs.");

namespace qcraft::segmentation {

namespace {

constexpr double kRoiLength = 10.0;
constexpr double kRoiWidth = 5.0;
constexpr double kMinInRoiRatio = 0.8;
constexpr int kRetroreflectorMinIntensity = 250;
constexpr int kMinNumRetroreflectedPoints = 5;
constexpr double kMinOusterPointRatio = 0.95;

ProposedClusters TrimReflectiveStripNoise(const ProposedCluster& cluster) {
  const auto& bbox = *cluster.bounding_box();
  Box2d lower_left_bounding_box(bbox.center(), bbox.heading(),
                                bbox.length() / 2, bbox.width() / 4);
  lower_left_bounding_box.Shift(Vec2d(-bbox.length() / 4, bbox.width() / 8 * 3)
                                    .FastRotate(bbox.heading()));

  ObstaclePtrs retroreflected_obstacles;
  for (const auto* obstacle : cluster.obstacles()) {
    if (!lower_left_bounding_box.IsPointIn(obstacle->coord())) {
      continue;
    }
    int num_retroreflected_points = 0;
    for (const auto& p : obstacle->points) {
      if (p.intensity >= kRetroreflectorMinIntensity) {
        ++num_retroreflected_points;
      }
    }
    if (num_retroreflected_points >= kMinNumRetroreflectedPoints) {
      retroreflected_obstacles.push_back(obstacle);
    }
  }

  if (retroreflected_obstacles.empty()) {
    return {cluster};
  }

  if (FLAGS_ouster_noise_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/TrimReflectiveStripNoise");
    for (const auto* obstacle : retroreflected_obstacles) {
      canvas.DrawPoint({obstacle->x, obstacle->y, 0}, vis::Color::kRed, 10);
    }
  }

  ObstaclePtrs vehicle_obstacles, trimmed_obstacles;
  for (const auto* obstacle : cluster.obstacles()) {
    if (bbox.IsPointIn(obstacle->coord())) {
      vehicle_obstacles.emplace_back(obstacle);
      continue;
    }
    Box2d serach_roi(obstacle->coord(), bbox.heading(), 1.8, 1.8);
    serach_roi.Shift(
        Vec2d(0.0, -serach_roi.half_width()).FastRotate(bbox.heading()));
    bool near_retroreflected_obstacle = false;
    for (const auto* retroreflected_obstacle : retroreflected_obstacles) {
      if (serach_roi.IsPointIn(retroreflected_obstacle->coord())) {
        near_retroreflected_obstacle = true;
        break;
      }
    }
    if (!near_retroreflected_obstacle) {
      vehicle_obstacles.emplace_back(obstacle);
      continue;
    }
    int num_ouster_points = 0;
    for (const auto& p : obstacle->points) {
      if (IsOusterPoint(p.lidar_type)) {
        ++num_ouster_points;
      }
    }
    if (num_ouster_points * 1.0 / obstacle->points.size() >
        kMinOusterPointRatio) {
      trimmed_obstacles.emplace_back(obstacle);
    } else {
      vehicle_obstacles.emplace_back(obstacle);
    }
  }

  if (trimmed_obstacles.empty()) {
    return {cluster};
  }

  if (FLAGS_ouster_noise_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/TrimReflectiveStripNoise");
    for (const auto* obstacle : trimmed_obstacles) {
      canvas.DrawPoint({obstacle->x, obstacle->y, 0}, vis::Color::kYellow, 10);
    }
  }

  ProposedClusters proposed_clusters;
  proposed_clusters.emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(trimmed_obstacles));
  proposed_clusters.back().set_is_proposed(true);
  proposed_clusters.back().set_property(PP_NOISE);

  proposed_clusters.emplace_back(
      ProposedCluster::InheritFrom(cluster).ConstructBase(vehicle_obstacles));
  proposed_clusters.back().set_is_proposed(true);
  proposed_clusters.back().set_bounding_box(*cluster.bounding_box());
  proposed_clusters.back().set_type(cluster.type());
  proposed_clusters.back().set_score(cluster.score());
  proposed_clusters.back().set_type_source(cluster.type_source());

  return proposed_clusters;
}

}  // namespace

ProposedClusters OusterNoiseProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  const auto& pose = env_info.pose();

  ProposedClusters proposed_clusters;

  Box2d roi_box({pose.x, pose.y}, pose.yaw, kRoiLength, kRoiWidth);
  roi_box.Shift(Vec2d(kRoiLength / 2, -kRoiWidth / 2).FastRotate(pose.yaw));

  Polygon2d roi_polygon(roi_box);
  for (const auto& cluster : clusters) {
    if (cluster.HasProperty(PP_NOISE)) {
      proposed_clusters.push_back(cluster);
      continue;
    }
    const auto& polygon = cluster_util::ComputeContour(cluster);
    Polygon2d intersection;
    bool is_in_roi = roi_polygon.ComputeOverlap(polygon, &intersection) &&
                     (intersection.area() / polygon.area() > kMinInRoiRatio);
    bool has_valid_bounding_box =
        cluster.type() == MT_VEHICLE && cluster.bounding_box() &&
        std::abs(pose.yaw - cluster.bounding_box()->heading()) < M_PI_4;
    if (is_in_roi && has_valid_bounding_box) {
      auto trimmed_clusters = TrimReflectiveStripNoise(cluster);
      for (auto& trimmed_cluster : trimmed_clusters) {
        proposed_clusters.push_back(std::move(trimmed_cluster));
      }
    } else {
      proposed_clusters.push_back(cluster);
    }
  }

  if (FLAGS_ouster_noise_proposer_cvs) {
    vis::Canvas& canvas =
        vantage_client_man::GetCanvas("perception/ouster_noise_proposer_cvs");
    canvas.DrawBox({roi_box.center_x(), roi_box.center_y(), 0.0},
                   roi_box.heading(), {roi_box.length(), roi_box.width()},
                   vis::Color::kGreen);
  }

  return proposed_clusters;
}

}  // namespace qcraft::segmentation
