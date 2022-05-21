#include "onboard/perception/cluster_util.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/math/test_util.h"
#include "onboard/perception/test_util/cluster_builder.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft::cluster_util {

TEST(ComputeContourTest, ShouldPass) {
  std::vector<Obstacle> obstacles = {
      ObstacleBuilder()
          .set_x(1.)
          .set_y(2.)
          .set_min_z(3.)
          .set_max_z(4.)
          .AdjustColRowGivenXy()
          .PopulatePointsRandomlyGivenObstacleInfo(10, 25, 25, 0)
          .Build(),
      ObstacleBuilder()
          .set_x(1.2)
          .set_y(2.)
          .set_min_z(3.)
          .set_max_z(4.)
          .AdjustColRowGivenXy()
          .PopulatePointsRandomlyGivenObstacleInfo(10, 25, 25, 0)
          .Build()};

  const Cluster cluster =
      ClusterBuilder()
          .set_obstacles(ConstructObstaclePtrsFromObstacles(obstacles))
          .Build();

  const auto contour = ComputeContour(cluster);
  EXPECT_EQ(contour.num_points(), 4);
  EXPECT_THAT(contour.points(),
              testing::ElementsAre(
                  Vec2dNearXY(0.9, 1.9, 1e-6), Vec2dNearXY(1.3, 1.9, 1e-6),
                  Vec2dNearXY(1.3, 2.1, 1e-6), Vec2dNearXY(0.9, 2.1, 1e-6)));
}

}  // namespace qcraft::cluster_util
