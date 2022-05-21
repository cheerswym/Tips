#ifndef ONBOARD_PERCEPTION_OBSTACLE_H_
#define ONBOARD_PERCEPTION_OBSTACLE_H_

#include <memory>
#include <vector>

#include "onboard/perception/laser_point.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

struct Obstacle;
using Obstacles = std::vector<Obstacle>;
using ObstacleRef = std::unique_ptr<Obstacle>;
using ObstacleRefVector = std::vector<ObstacleRef>;
using ObstaclePtr = const Obstacle*;
using ObstaclePtrs = std::vector<ObstaclePtr>;

struct Obstacle {
  static constexpr float kDiameter = 0.2;  // m

  static std::vector<Obstacle> ObstaclesFromProto(const ObstaclesProto& proto);
  static ObstaclesProto ObstaclesToProto(const ObstaclePtrs& obstacles);

  Vec2d coord() const { return {x, y}; }
  float height() const { return max_z - ground_z; }
  int above_ground_points_start_index() const {
    return points.size() - num_points_above_ground;
  }
  // Initialized by point timestamp.
  double timestamp = 0.0;
  float x = 0.0f;
  float y = 0.0f;
  float min_z = 0.0f;
  float max_z = 0.0f;
  float ground_z = 0.0f;
  // The distance between the ground and the lowest point above ground.
  float clearance = 0.0f;

  uint16_t col = 0;  // Corresponds to x in smooth coord.
  uint16_t row = 0;  // Corresponds to y in smooth coord.

  // TODO(yu): Better make use of dynamic obstacle vs. static obstacle.
  ObstacleProto::Type type = ObstacleProto::STATIC;

  ObstacleProto::TypeSource type_source = ObstacleProto::DEFAULT;

  // The distance to curb. Negative means on road and positive means offroad.
  float dist_to_curb = 0.0f;

  // If this obstacle is likely mist (or smoke/dust/vapor/rain/fog/etc).
  bool is_likely_mist = false;

  // Score indicating how likely the current obstacle is mist.
  float mist_score = -1.f;

  // Num points above ground.
  int num_points_above_ground = 0;

  // Points are sorted by their z-coordinate.
  std::vector<LaserPoint> points;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_OBSTACLE_H_
