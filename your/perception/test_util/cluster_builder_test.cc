#include "onboard/perception/test_util/cluster_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/perception/test_util/obstacle_builder.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace perception {
namespace {

TEST(ClusterBuilderTest, Build) {
  const Cluster cluster = ClusterBuilder().Build();
  EXPECT_EQ(cluster.type(), MT_UNKNOWN);
  EXPECT_EQ(cluster.obstacles().size(), 0);
}

TEST(ClusterBuilderTest, SetType) {
  const Cluster cluster = ClusterBuilder().set_type(MT_CONE).Build();
  EXPECT_EQ(cluster.type(), MT_CONE);
}

TEST(ClusterBuilderTest, SetObstacles) {
  std::vector<Obstacle> obstacles = {
      ObstacleBuilder()
          .set_x(1.)
          .set_y(2.)
          .set_min_z(3.)
          .set_max_z(4.)
          .PopulatePointsRandomlyGivenObstacleInfo(10, 25, 25, 0)
          .Build(),
      ObstacleBuilder()
          .set_x(1.2)
          .set_y(2.)
          .set_min_z(3.)
          .set_max_z(4.)
          .PopulatePointsRandomlyGivenObstacleInfo(10, 25, 25, 0)
          .Build()};

  const Cluster cluster =
      ClusterBuilder()
          .set_obstacles(ConstructObstaclePtrsFromObstacles(obstacles))
          .Build();
  EXPECT_EQ(cluster.obstacles().size(), 2);
}

}  // namespace
}  // namespace perception
}  // namespace qcraft
