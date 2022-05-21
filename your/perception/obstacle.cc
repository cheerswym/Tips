#include "onboard/perception/obstacle.h"

#include <algorithm>

#include "glog/logging.h"
#include "onboard/global/trace.h"

namespace qcraft {

std::vector<Obstacle> Obstacle::ObstaclesFromProto(
    const ObstaclesProto& proto) {
  // For V2.
  std::vector<Obstacle> obstacles;
  if (proto.compact_obstacles_size() > 0) {
    obstacles.reserve(proto.compact_obstacles_size());
    const Vec3d origin(proto.compact_obstacle_info().origin().x(),
                       proto.compact_obstacle_info().origin().y(),
                       proto.compact_obstacle_info().origin().z());
    for (const auto& obstacle_proto : proto.compact_obstacles()) {
      obstacles.emplace_back();
      auto& obstacle = obstacles.back();
      obstacle.x = origin.x() + Obstacle::kDiameter * obstacle_proto.x_index();
      obstacle.y = origin.y() + Obstacle::kDiameter * obstacle_proto.y_index();
      obstacle.min_z = origin.z() + obstacle_proto.min_z_cm() * 0.01f;
      obstacle.max_z = origin.z() + obstacle_proto.max_z_cm() * 0.01f;
      obstacle.ground_z = origin.z() + obstacle_proto.ground_z_cm() * 0.01f;
      obstacle.clearance = obstacle_proto.clearance_cm() * 0.01f;
      obstacle.type = obstacle_proto.type();
      obstacle.type_source = obstacle_proto.type_source();
      obstacle.mist_score = obstacle_proto.mist_score() * 0.01f;
      obstacle.dist_to_curb = obstacle_proto.dist_to_curb_cm() * 0.01f;
      obstacle.num_points_above_ground =
          obstacle_proto.num_points_above_ground();
    }
  } else {
    obstacles.reserve(proto.obstacles_size());
    for (const auto& obstacle_proto : proto.obstacles()) {
      obstacles.emplace_back();
      auto& obstacle = obstacles.back();
      obstacle.x = obstacle_proto.x();
      obstacle.y = obstacle_proto.y();
      obstacle.min_z = obstacle_proto.min_z();
      obstacle.max_z = obstacle_proto.max_z();
      obstacle.ground_z = obstacle_proto.ground_z();
      obstacle.clearance = obstacle_proto.clearance();
      obstacle.type = obstacle_proto.type();
      obstacle.type_source = obstacle_proto.type_source();
      obstacle.mist_score = obstacle_proto.mist_score();
      obstacle.dist_to_curb = obstacle_proto.dist_to_curb();
      obstacle.num_points_above_ground =
          obstacle_proto.num_points_above_ground();
    }
  }
  return obstacles;
}

ObstaclesProto Obstacle::ObstaclesToProto(const ObstaclePtrs& obstacles) {
  SCOPED_QTRACE_ARG1("Obstacle::ObstaclesToProto", "num_obstacles",
                     obstacles.size());
  ObstaclesProto proto;
  double timestamp_sum = 0;
  Vec3f origin(FLT_MAX, FLT_MAX, FLT_MAX);
  for (const auto* obstacle : obstacles) {
    origin.x() = std::min(origin.x(), obstacle->x);
    origin.y() = std::min(origin.y(), obstacle->y);
    origin.z() =
        std::min(origin.z(), std::min(obstacle->min_z, obstacle->ground_z));
  }
  proto.mutable_compact_obstacle_info()->set_diameter(Obstacle::kDiameter);
  proto.mutable_compact_obstacle_info()->mutable_origin()->set_x(origin.x());
  proto.mutable_compact_obstacle_info()->mutable_origin()->set_y(origin.y());
  proto.mutable_compact_obstacle_info()->mutable_origin()->set_z(origin.z());

  for (const auto* obstacle : obstacles) {
    auto* obstacle_proto = proto.add_compact_obstacles();
    obstacle_proto->set_x_index(
        RoundToInt((obstacle->x - origin.x()) * (1.0f / Obstacle::kDiameter)));
    obstacle_proto->set_y_index(
        RoundToInt((obstacle->y - origin.y()) * (1.0f / Obstacle::kDiameter)));
    obstacle_proto->set_min_z_cm(
        RoundToInt((obstacle->min_z - origin.z()) * 100.f));
    obstacle_proto->set_max_z_cm(
        RoundToInt((obstacle->max_z - origin.z()) * 100.f));
    obstacle_proto->set_ground_z_cm(
        RoundToInt((obstacle->ground_z - origin.z()) * 100.f));
    obstacle_proto->set_num_points(obstacle->points.size());
    obstacle_proto->set_num_points_above_ground(
        obstacle->num_points_above_ground);
    obstacle_proto->set_type(obstacle->type);
    obstacle_proto->set_clearance_cm(RoundToInt(obstacle->clearance * 100.f));
    obstacle_proto->set_mist_score(RoundToInt(obstacle->mist_score * 100.f));
    obstacle_proto->set_type_source(obstacle->type_source);
    obstacle_proto->set_dist_to_curb_cm(
        RoundToInt(obstacle->dist_to_curb * 100.f));
    timestamp_sum += obstacle->timestamp;
  }
  proto.set_timestamp(obstacles.size() == 0 ? 0
                                            : timestamp_sum / obstacles.size());
  return proto;
}

}  // namespace qcraft
