#include "onboard/perception/obstacle.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft {

TEST(ObstacleTest, TestCoord) {
  EXPECT_EQ(ObstacleBuilder()
                .set_x(1.0)
                .set_y(1.0)
                .set_ground_z(1.0)
                .set_points({{.range = 0.}})
                .Build()
                .coord(),
            Vec2d({1.0, 1.0}));
}

TEST(ObstacleTest, TestHeight) {
  EXPECT_NEAR(ObstacleBuilder()
                  .set_x(1.0)
                  .set_y(1.0)
                  .set_max_z(2.0)
                  .set_ground_z(1.0)
                  .set_points({{.range = 0.}})
                  .Build()
                  .height(),
              1.0, 1e-6);
}

TEST(ObstacleTest, TestAboveGroundPointStartIndex) {
  EXPECT_EQ(ObstacleBuilder()
                .set_x(1.0)
                .set_y(1.0)
                .set_min_z(0.0)
                .set_max_z(2.0)
                .set_ground_z(1.0)
                .set_num_points_above_ground(5)
                .PopulatePointsRandomlyGivenObstacleInfo(10, 25, 25, 0)
                .Build()
                .above_ground_points_start_index(),
            5);
}

}  // namespace qcraft
