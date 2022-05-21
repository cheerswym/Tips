#include "onboard/perception/test_util/obstacle_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace perception {
namespace {

TEST(ObstacleBuilder, Build) {
  const Obstacle obstacle = ObstacleBuilder().Build();
  EXPECT_EQ(obstacle.points.size(), 1);
  EXPECT_EQ(obstacle.min_z, 0.);
}

TEST(ObstacleBuilder, BuildAndPopulateAll) {
  const Obstacle obstacle = ObstacleBuilder()
                                .set_x(1.)
                                .set_y(2.)
                                .set_min_z(3.)
                                .set_max_z(4.)
                                .set_points({{.z = 0}})
                                .AdjustColRowGivenXy()
                                .Build();
  EXPECT_EQ(obstacle.points.size(), 1);
  EXPECT_EQ(obstacle.min_z, 3.);
  EXPECT_EQ(obstacle.col, 5);
}

TEST(ObstacleBuilder, BuildAndPopulatePointsRandomlyGivenObstacleInfo) {
  const Obstacle obstacle =
      ObstacleBuilder()
          .set_x(1.)
          .set_y(2.)
          .set_min_z(3.)
          .set_max_z(4.)
          .PopulatePointsRandomlyGivenObstacleInfo(10, 25, 25, 0)
          .Build();
  EXPECT_EQ(obstacle.points.size(), 10);
  EXPECT_EQ(obstacle.points.front().z, 3.);
  EXPECT_EQ(obstacle.points.back().z, 4.);
  EXPECT_LE(obstacle.points.front().z, obstacle.points.back().z);
  EXPECT_EQ(obstacle.min_z, 3.);
}

}  // namespace
}  // namespace perception
}  // namespace qcraft
