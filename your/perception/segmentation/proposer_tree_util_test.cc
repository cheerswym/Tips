#include "onboard/perception/segmentation/proposer_tree_util.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft::segmentation {

TEST(ProposerTreeUtilTest, TestBuildProposerTree) {
  const ProposerTree proposer_tree(BuildProposerTree(nullptr));

  const auto tree_nodes_proto = proposer_tree.ToProto();

  EXPECT_EQ(tree_nodes_proto.tree_nodes().size(), 13);

  for (const auto& tree_node : tree_nodes_proto.tree_nodes()) {
    if (tree_node.name() == "PT_FIERY_EYE_NET") {
      EXPECT_EQ(tree_node.parent(), "PT_NONE");
    } else if (tree_node.name() == "PT_SEMANTIC_MAP") {
      EXPECT_EQ(tree_node.parent(), "PT_FIERY_EYE_NET");
    } else if (tree_node.name() == "PT_LL_NET") {
      EXPECT_EQ(tree_node.parent(), "PT_SEMANTIC_MAP");
    } else if (tree_node.name() == "PT_OUSTER_NOISE") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_CATEYE") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_BLOOMING") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_BARRIER") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_LARGE_VEHICLE") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_LEAF") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_REFLECTION") {
      EXPECT_EQ(tree_node.parent(), "PT_BLOOMING");
    } else if (tree_node.name() == "PT_SPLASH") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_BPEARL_NOISE") {
      EXPECT_EQ(tree_node.parent(), "PT_LL_NET");
    } else if (tree_node.name() == "PT_DISTANT_CLUSTER") {
      EXPECT_EQ(tree_node.parent(), "PT_REFLECTION");
    } else {
      FAIL() << "should not reach here.";
    }
  }
}

TEST(ProposerTreeUtilTest, TestProposerTreeTraverse) {
  const ProposerTree proposer_tree(BuildProposerTree(nullptr));
  auto func = [&](ProposerTreeNodePtr node) {
    switch (node->type()) {
      case ProposerType::PT_FIERY_EYE_NET:
        EXPECT_EQ(node->score(), 9);
        EXPECT_EQ(node->depth(), 1);
        break;
      case ProposerType::PT_LL_NET:
        EXPECT_EQ(node->score(), 9);
        EXPECT_EQ(node->depth(), 3);
        break;
      case ProposerType::PT_OUSTER_NOISE:
        EXPECT_EQ(node->score(), 1);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_CATEYE:
        EXPECT_EQ(node->score(), 5);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_BLOOMING:
        EXPECT_EQ(node->score(), 8);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_SEMANTIC_MAP:
        EXPECT_EQ(node->score(), 9);
        EXPECT_EQ(node->depth(), 2);
        break;
      case ProposerType::PT_BARRIER:
        EXPECT_EQ(node->score(), 6);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_LARGE_VEHICLE:
        EXPECT_EQ(node->score(), 9);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_LEAF:
        EXPECT_EQ(node->score(), 4);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_REFLECTION:
        EXPECT_EQ(node->score(), 9);
        EXPECT_EQ(node->depth(), 5);
        break;
      case ProposerType::PT_SPLASH:
        EXPECT_EQ(node->score(), 7);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_BPEARL_NOISE:
        EXPECT_EQ(node->score(), 2);
        EXPECT_EQ(node->depth(), 4);
        break;
      case ProposerType::PT_DISTANT_CLUSTER:
        EXPECT_EQ(node->score(), 9);
        EXPECT_EQ(node->depth(), 6);
        break;
      case ProposerType::PT_NONE:
        EXPECT_EQ(node->score(), 0);
        EXPECT_EQ(node->depth(), 0);
        break;
      default:
        FAIL() << "should not reach here.";
    }
  };
  proposer_tree.Traverse(std::move(func));
}

}  // namespace qcraft::segmentation
