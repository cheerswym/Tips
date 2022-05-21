#include "onboard/perception/test_util/obstacle_builder.h"

#include <algorithm>
#include <random>
#include <utility>

#include "onboard/perception/laser_point.h"
#include "onboard/proto/lidar.pb.h"

namespace qcraft {

ObstaclePtrs ConstructObstaclePtrsFromObstacles(const Obstacles& obstacles) {
  ObstaclePtrs obstacle_ptrs;
  for (const auto& obstacle : obstacles) {
    obstacle_ptrs.push_back(&obstacle);
  }
  return obstacle_ptrs;
}

ObstaclePtrs ConstructObstaclePtrsFromObstacleRefVector(
    const ObstacleRefVector& obstacle_refs) {
  ObstaclePtrs obstacle_ptrs;
  for (const auto& obstacle_ref : obstacle_refs) {
    obstacle_ptrs.push_back(obstacle_ref.get());
  }
  return obstacle_ptrs;
}

ObstacleBuilder::ObstacleBuilder() {
  obstacle_.timestamp = 0.0;
  obstacle_.x = 0.0f;
  obstacle_.y = 0.0f;
  obstacle_.min_z = 0.0f;
  obstacle_.max_z = 0.0f;
  obstacle_.ground_z = 0.0f;
  obstacle_.clearance = 0.0f;
  obstacle_.row = 0;
  obstacle_.col = 0;
  obstacle_.type = ObstacleProto::STATIC;
  obstacle_.type_source = ObstacleProto::DEFAULT;
  obstacle_.dist_to_curb = 1.0f;
  obstacle_.is_likely_mist = false;
  obstacle_.mist_score = -1.f;
  obstacle_.num_points_above_ground = 0;
  obstacle_.points.push_back({.z = 0.0f});
}

ObstacleBuilder& ObstacleBuilder::set_x(float x) {
  obstacle_.x = x;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_y(float y) {
  obstacle_.y = y;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_min_z(float min_z) {
  obstacle_.min_z = min_z;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_max_z(float max_z) {
  obstacle_.max_z = max_z;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_ground_z(float ground_z) {
  obstacle_.ground_z = ground_z;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_col(uint16_t col) {
  obstacle_.col = col;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_row(uint16_t row) {
  obstacle_.row = row;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_type(ObstacleProto::Type type) {
  obstacle_.type = type;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_type_source(
    ObstacleProto::TypeSource type_source) {
  obstacle_.type_source = type_source;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_num_points_above_ground(
    int num_points_above_ground) {
  obstacle_.num_points_above_ground = num_points_above_ground;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::set_points(std::vector<LaserPoint>&& points) {
  obstacle_.points = std::move(points);
  return *this;
}

ObstacleBuilder& ObstacleBuilder::AdjustColRowGivenXy() {
  constexpr float kDiameterReversed = 1. / Obstacle::kDiameter;
  obstacle_.col = obstacle_.x * kDiameterReversed;
  obstacle_.row = obstacle_.y * kDiameterReversed;
  return *this;
}

ObstacleBuilder& ObstacleBuilder::PopulatePointsRandomlyGivenObstacleInfo(
    int num_points) {
  return PopulatePointsRandomlyGivenObstacleInfo(num_points, 0, 0, 0.);
}

ObstacleBuilder& ObstacleBuilder::PopulatePointsRandomlyGivenObstacleInfo(
    int num_points, uint8_t intensity, uint8_t intensity_var, float range) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0., 1.);  // in [0., 1).
  QCHECK_LE(obstacle_.min_z, obstacle_.max_z);
  // num_points has to be greater than 1 if min_z/max_z are different.
  QCHECK(obstacle_.min_z == obstacle_.max_z && num_points >= 1 ||
         obstacle_.min_z < obstacle_.max_z && num_points > 1);

  obstacle_.points.clear();

  const auto generate_a_point_given_intensity =
      [&](uint8_t intensity) -> LaserPoint {
    const float x =
        obstacle_.x + static_cast<float>(dis(gen) - 0.5) * Obstacle::kDiameter;
    const float y =
        obstacle_.y + static_cast<float>(dis(gen) - 0.5) * Obstacle::kDiameter;
    const float z = obstacle_.min_z + static_cast<float>(dis(gen)) *
                                          (obstacle_.max_z - obstacle_.min_z);
    return {.timestamp = 0.0,
            .x = x,
            .y = y,
            .z = z,
            .range = 0.0f,
            .normal_x = 0,
            .normal_y = 0,
            .normal_z = 0,
            .has_return_behind = false,
            .intensity = intensity,
            .planarity = 0,
            .beam_index = 0,
            .return_index = 0,
            .scan_or_point_index = 0,
            .lidar_id = LDR_CENTER,
            .lidar_type = LIDAR_PANDAR_40M};
  };

  for (int i = 0; i < num_points; ++i) {
    const uint8_t point_intensity = std::max(
        0.f, intensity + static_cast<float>(dis(gen) * 2. - 1) * intensity_var);
    obstacle_.points.push_back(
        generate_a_point_given_intensity(point_intensity));
  }

  std::sort(obstacle_.points.begin(), obstacle_.points.end(),
            [](const auto& e1, const auto& e2) { return e1.z < e2.z; });
  obstacle_.points.front().z = obstacle_.min_z;
  obstacle_.points.back().z = obstacle_.max_z;

  return *this;
}

Obstacle ObstacleBuilder::Build() { return obstacle_; }

}  // namespace qcraft
