#include "onboard/perception/segmentation/proposer_util.h"

#include <algorithm>
#include <cstdint>
#include <set>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/math/util.h"
#include "onboard/perception/segmentation/proposed_cluster.h"

namespace qcraft::segmentation {

TEST(ProposerUtilTest, TestSegmentAndProposeClusterWithConnectedComponents) {
  /*
  | | | | | | | | |
  | | | | |o|o|o|o|
  | | | | |o|o|o|o|
  | | | | |o|o|o|o|
  | | | |x|o|o|o|o|
  |*|*|*| | | | | |
  |*|*|*| | | | | |
  |*|*|*| | | | | |
  */
  ObstaclePtrs obstacles;
  for (uint16_t i = 0; i < 3; ++i) {
    for (uint16_t j = 0; j < 3; ++j) {
      obstacles.emplace_back(
          new Obstacle{.col = j, .row = i, .type = ObstacleProto::STATIC});
    }
  }
  for (uint16_t i = 3; i < 7; ++i) {
    for (uint16_t j = 4; j < 8; ++j) {
      obstacles.emplace_back(
          new Obstacle{.col = j, .row = i, .type = ObstacleProto::DYNAMIC});
    }
  }
  obstacles.emplace_back(
      new Obstacle{.col = 3, .row = 3, .type = ObstacleProto::CONE});

  ProposedClusters proposed_clusters =
      InitProposedClusters({Cluster(obstacles)});
  auto segmented_clusters = SegmentAndProposeClusterWithConnectedComponents(
      proposed_clusters[0], 4,
      [](const Obstacle& init, const Obstacle& seed, const Obstacle& neighbor) {
        return init.type == neighbor.type;
      });
  EXPECT_EQ(3, segmented_clusters.size());
  segmented_clusters =
      SegmentAndProposeClusterWithConnectedComponents(proposed_clusters[0], 4);
  EXPECT_EQ(2, segmented_clusters.size());
  segmented_clusters =
      SegmentAndProposeClusterWithConnectedComponents(proposed_clusters[0], 8);
  EXPECT_EQ(1, segmented_clusters.size());
  segmented_clusters = SegmentAndProposeClusterWithConnectedComponents(
      proposed_clusters[0], 8,
      [](const Obstacle& init, const Obstacle& seed, const Obstacle& neighbor) {
        return init.type == neighbor.type;
      });
  EXPECT_EQ(3, segmented_clusters.size());

  std::set<ObstaclePtr> segmented_obstacle_set;
  for (const auto& cluster : segmented_clusters) {
    EXPECT_EQ(cluster.is_proposed(), true);
    for (const auto* obstacle : cluster.obstacles()) {
      EXPECT_EQ(segmented_obstacle_set.insert(obstacle).second, true);
    }
  }
  std::set<ObstaclePtr> obstacle_set;
  for (const auto* obstacle : obstacles) {
    EXPECT_EQ(obstacle_set.insert(obstacle).second, true);
  }

  EXPECT_EQ(segmented_obstacle_set.size(), obstacle_set.size());

  auto it1 = segmented_obstacle_set.begin();
  auto it2 = obstacle_set.begin();

  while (it1 != segmented_obstacle_set.end()) {
    EXPECT_EQ(*it1, *it2);
    ++it1;
    ++it2;
  }

  for (auto* obstacle : obstacles) {
    delete obstacle;
  }
}

TEST(ProposerUtilTest, TestComputeClusterBoundingBoxMeanHeading) {
  std::vector<double> angles = {1.1};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles), 1.1);

  angles = {1.1, 1.2};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles), MeanAngle(angles));

  angles = {0.0, M_PI};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles), M_PI);

  angles = {0.0, 1.2, 1.5};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles), MeanAngle(angles));

  angles = {0.0, 2.7, 2.8, 2.9};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles),
            MeanAngle({M_PI, 2.7, 2.8, 2.9}));

  angles = {0.7, 1.4, 2.5, 3.1, 2.8, 2.9};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles),
            MeanAngle({(0.7 + M_PI), (1.4 + M_PI), 2.5, 3.1, 2.8, 2.9}));

  angles = {6.1, 6.2, 0.1, 0.2, 3.1, 2.8, 2.9};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles),
            MeanAngle({6.1, 6.2, 0.1, 0.2, (3.1 + M_PI), (2.8 + M_PI),
                       (2.9 + M_PI)}));

  angles = {-3.1, -3.0, 3.1, 3.0, 0.0, 0.1, 0.2};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles),
            MeanAngle({-3.1, -3.0, 3.1, 3.0, (0.0 + M_PI), (0.1 + M_PI),
                       (0.2 + M_PI)}));

  angles = {-3.1, -3, 3, 3.1};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles),
            MeanAngle({-3.1, -3, 3, 3.1}));

  angles = {1.0, 1.0};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles), MeanAngle(angles));

  angles = {2.0, 2.0, 2.0};
  EXPECT_EQ(ComputeClusterBoundingBoxMeanHeading(angles), MeanAngle(angles));
}

}  // namespace qcraft::segmentation
