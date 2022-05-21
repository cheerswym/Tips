#include "onboard/perception/segmentation/proposed_cluster_dag.h"

#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/segmentation/scene_map_test_util.h"

namespace qcraft::segmentation {

namespace {
ProposedClusterPtrs CollectProposedClusterPtrs(
    const ProposedClusters& clusters) {
  ProposedClusterPtrs cluster_ptrs;
  cluster_ptrs.reserve(clusters.size());
  for (const auto& cluster : clusters) {
    cluster_ptrs.emplace_back(&cluster);
  }
  return cluster_ptrs;
}

}  // namespace

TEST(ProposedClusterDagTest, TestComputeBigraph) {
  SceneMap scene_map = BuildSceneMap();
  ProposedClusterDag dag(scene_map);

  auto bigraph1 = dag.ComputeBigraph(
      PT_NONE, PT_FIERY_EYE_NET,
      CollectProposedClusterPtrs(scene_map.GetResult(PT_FIERY_EYE_NET)));
  EXPECT_EQ(bigraph1.parent_type, PT_NONE);
  EXPECT_EQ(bigraph1.child_type, PT_FIERY_EYE_NET);
  EXPECT_EQ(bigraph1.parent_to_childs_map.size(), 10);
  for (auto it = bigraph1.parent_to_childs_map.begin();
       it != bigraph1.parent_to_childs_map.end(); ++it) {
    EXPECT_EQ(it->second.size(), 2);
  }
  for (auto it = bigraph1.child_to_parents_map.begin();
       it != bigraph1.child_to_parents_map.end(); ++it) {
    EXPECT_EQ(it->second.size(), 1);
  }

  auto bigraph2 = dag.ComputeBigraph(
      PT_NONE, PT_SEMANTIC_MAP,
      CollectProposedClusterPtrs(scene_map.GetResult(PT_SEMANTIC_MAP)));
  EXPECT_EQ(bigraph2.parent_type, PT_NONE);
  EXPECT_EQ(bigraph2.child_type, PT_SEMANTIC_MAP);
  for (auto it = bigraph2.parent_to_childs_map.begin();
       it != bigraph2.parent_to_childs_map.end(); ++it) {
    EXPECT_EQ(it->second.size(), 1);
  }
  for (auto it = bigraph2.child_to_parents_map.begin();
       it != bigraph2.child_to_parents_map.end(); ++it) {
    EXPECT_EQ(it->second.size(), 2);
  }
}

TEST(ProposedClusterDagTest, TestGroupPacks) {
  SceneMap scene_map = BuildSceneMap();
  ProposedClusterDag dag(scene_map);

  auto bigraph1 = dag.ComputeBigraph(
      PT_NONE, PT_FIERY_EYE_NET,
      CollectProposedClusterPtrs(scene_map.GetResult(PT_FIERY_EYE_NET)));
  auto pack1 = GroupPacks(bigraph1);
  EXPECT_EQ(pack1.size(), 10);
  EXPECT_EQ(CalculateTotalNumParentClusters(pack1), 10);
  EXPECT_EQ(CalculateTotalNumChildClusters(pack1), 20);
  auto bigraph2 = dag.ComputeBigraph(
      PT_NONE, PT_SEMANTIC_MAP,
      CollectProposedClusterPtrs(scene_map.GetResult(PT_SEMANTIC_MAP)));
  auto pack2 = GroupPacks(bigraph2);
  EXPECT_EQ(pack2.size(), 5);
  EXPECT_EQ(CalculateTotalNumParentClusters(pack2), 10);
  EXPECT_EQ(CalculateTotalNumChildClusters(pack2), 5);
}

TEST(ProposedClusterDagTest, TestSelectClustersFromPacks) {
  SceneMap scene_map = BuildSceneMap();
  ProposedClusterDag dag(scene_map);

  auto bigraph1 = dag.ComputeBigraph(
      PT_NONE, PT_FIERY_EYE_NET,
      CollectProposedClusterPtrs(scene_map.GetResult(PT_FIERY_EYE_NET)));
  auto pack1 = GroupPacks(bigraph1);
  auto bigraph2 = dag.ComputeBigraph(
      PT_NONE, PT_SEMANTIC_MAP,
      CollectProposedClusterPtrs(scene_map.GetResult(PT_SEMANTIC_MAP)));
  auto pack2 = GroupPacks(bigraph2);
  pack1.insert(pack1.end(), pack2.begin(), pack2.end());
  auto selected_clusters = SelectClustersFromPacks(pack1);
  EXPECT_EQ(selected_clusters.size(), 14);
  std::map<double, int> num_clusters_of_score;
  for (const auto* selected_cluster : selected_clusters) {
    ++num_clusters_of_score[selected_cluster->proposer_score()];
  }
  EXPECT_EQ(num_clusters_of_score[0.5], 10);
  EXPECT_EQ(num_clusters_of_score[0.1], 2);
  EXPECT_EQ(num_clusters_of_score[0.0], 2);
}

}  // namespace qcraft::segmentation
