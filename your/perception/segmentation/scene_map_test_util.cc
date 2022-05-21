#include "onboard/perception/segmentation/scene_map_test_util.h"

#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/test_util/cluster_builder.h"
#include "onboard/perception/test_util/obstacle_builder.h"

namespace qcraft::segmentation {

SceneMap BuildSceneMap() {
  constexpr int kNumTotalObstacles = 100;
  std::vector<Obstacle> obstacles;
  obstacles.reserve(kNumTotalObstacles);
  for (uint16_t i = 0; i < kNumTotalObstacles; ++i) {
    obstacles.push_back(ObstacleBuilder().set_col(i).set_row(i).Build());
  }

  constexpr int kNumParentClusters = 10;
  ClusterVector parent_clusters;
  for (int i = 0; i < kNumParentClusters; ++i) {
    ObstaclePtrs obstacle_ptrs;
    for (int j = 0; j < kNumTotalObstacles / kNumParentClusters; ++j) {
      obstacle_ptrs.emplace_back(
          &obstacles[i * kNumTotalObstacles / kNumParentClusters + j]);
    }
    parent_clusters.emplace_back(std::move(obstacle_ptrs));
  }
  ProposedClusters proposed_clusters =
      InitProposedClusters(std::move(parent_clusters));

  constexpr int kNumChildClusters1 = 20;
  ClusterVector child_clusters_1;
  for (int i = 0; i < kNumChildClusters1; ++i) {
    ObstaclePtrs obstacle_ptrs;
    for (int j = 0; j < kNumTotalObstacles / kNumChildClusters1; ++j) {
      obstacle_ptrs.emplace_back(
          &obstacles[i * kNumTotalObstacles / kNumChildClusters1 + j]);
    }
    child_clusters_1.emplace_back(std::move(obstacle_ptrs));
  }
  ProposedClusters proposed_clusters_1 =
      InitProposedClusters(std::move(child_clusters_1));
  for (int i = 0; i < kNumChildClusters1 / 2; ++i) {
    proposed_clusters_1[i].set_proposer_score(0.5);
  }

  constexpr int kNumChildClusters2 = 5;
  ClusterVector child_clusters_2;
  for (int i = 0; i < kNumChildClusters2; ++i) {
    ObstaclePtrs obstacle_ptrs;
    for (int j = 0; j < kNumTotalObstacles / kNumChildClusters2; ++j) {
      obstacle_ptrs.emplace_back(
          &obstacles[i * kNumTotalObstacles / kNumChildClusters2 + j]);
    }
    child_clusters_2.emplace_back(std::move(obstacle_ptrs));
  }
  ProposedClusters proposed_clusters_2 =
      InitProposedClusters(std::move(child_clusters_2));
  for (int i = 0; i < kNumChildClusters2; ++i) {
    proposed_clusters_2[i].set_proposer_score(0.1);
  }

  SceneMap scene_map;
  scene_map.emplace(PT_NONE, proposed_clusters);
  scene_map.emplace(PT_FIERY_EYE_NET, proposed_clusters_1);
  scene_map.emplace(PT_SEMANTIC_MAP, proposed_clusters_2);

  return scene_map;
}

}  // namespace qcraft::segmentation
