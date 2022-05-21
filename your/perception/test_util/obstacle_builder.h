#ifndef ONBOARD_PERCEPTION_TEST_UTIL_OBSTACLE_BUILDER_H_
#define ONBOARD_PERCEPTION_TEST_UTIL_OBSTACLE_BUILDER_H_

#include <vector>

#include "onboard/perception/obstacle.h"

namespace qcraft {

// Creates a obstacle, might populate points according to information in
// obstacle struct randomly.
class ObstacleBuilder {
 public:
  ObstacleBuilder();

  ObstacleBuilder& set_x(float x);
  ObstacleBuilder& set_y(float y);
  ObstacleBuilder& set_min_z(float min_z);
  ObstacleBuilder& set_max_z(float max_z);
  ObstacleBuilder& set_ground_z(float ground_z);
  ObstacleBuilder& set_col(uint16_t col);
  ObstacleBuilder& set_row(uint16_t row);
  ObstacleBuilder& set_type(ObstacleProto::Type type);
  ObstacleBuilder& set_type_source(ObstacleProto::TypeSource type_source);
  ObstacleBuilder& set_num_points_above_ground(int num_points_above_ground);

  ObstacleBuilder& set_points(std::vector<LaserPoint>&& points);

  // Fills in col and row automatically according to obstacle x y. The
  // reason to have this function is due to the fact that some functionalities
  // assuming col/row is computed already by givng x/y.
  ObstacleBuilder& AdjustColRowGivenXy();

  // Populates points according to obstacle info (min_z, max_z etc.).
  ObstacleBuilder& PopulatePointsRandomlyGivenObstacleInfo(
      int num_points, uint8_t intensity, uint8_t intensity_var, float range);

  ObstacleBuilder& PopulatePointsRandomlyGivenObstacleInfo(int num_points);

  Obstacle Build();

 private:
  Obstacle obstacle_;
};

ObstaclePtrs ConstructObstaclePtrsFromObstacles(const Obstacles& obstacles);
ObstaclePtrs ConstructObstaclePtrsFromObstacleRefVector(
    const ObstacleRefVector& obstacle_refs);

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TEST_UTIL_OBSTACLE_BUILDER_H_
