#include "onboard/perception/segmentation/pyramid.h"

#include <string>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/segmentation/proposer_tree_util.h"
#include "onboard/perception/segmentation/scene_map_test_util.h"

namespace qcraft::segmentation {

TEST(PyramidTest, TestBuildClusterPyramid) {
  const std::string conf_path =
      "onboard/perception/segmentation/conf/proposer_tree_test.pb.txt";
  const ProposerTree proposer_tree(BuildProposerTree(nullptr, conf_path));
  SceneMap scene_map = BuildSceneMap();
  const ProposedClusterDag dag(scene_map);
  Pyramid pyramid(proposer_tree);
  pyramid.BuildClusterPyramid(scene_map, dag);
  Pyramid::ClusterPyramid cluster_pyramid = pyramid.cluster_pyramid();
  EXPECT_EQ(cluster_pyramid.size(), 2);
  EXPECT_NE(FindOrNull(cluster_pyramid, 0), nullptr);
  EXPECT_EQ(cluster_pyramid[0].size(), 1);
  for (const auto& [type, clusters] : cluster_pyramid[0]) {
    EXPECT_EQ(type, ProposerType::PT_NONE);
    EXPECT_EQ(clusters->size(), 14);
  }
  EXPECT_NE(FindOrNull(cluster_pyramid, 1), nullptr);
  EXPECT_EQ(cluster_pyramid[1].size(), 2);
  EXPECT_NE(FindOrNull(cluster_pyramid[1], ProposerType::PT_FIERY_EYE_NET),
            nullptr);
  EXPECT_NE(FindOrNull(cluster_pyramid[1], ProposerType::PT_SEMANTIC_MAP),
            nullptr);
  const auto clusters_fen = cluster_pyramid[1][ProposerType::PT_FIERY_EYE_NET];
  EXPECT_EQ(clusters_fen->size(), 20);
  const auto clusters_sm = cluster_pyramid[1][ProposerType::PT_SEMANTIC_MAP];
  EXPECT_EQ(clusters_sm->size(), 5);
}

}  // namespace qcraft::segmentation
