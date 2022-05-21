#include "onboard/perception/segmentation/proposer_tree_util.h"

#include <memory>
#include <string>
#include <vector>

#include "onboard/perception/segmentation/barrier_proposer.h"
#include "onboard/perception/segmentation/blooming_proposer.h"
#include "onboard/perception/segmentation/bpearl_noise_proposer.h"
#include "onboard/perception/segmentation/cateye_proposer.h"
#include "onboard/perception/segmentation/distant_cluster_proposer.h"
#include "onboard/perception/segmentation/fiery_eye_net_proposer.h"
#include "onboard/perception/segmentation/large_vehicle_proposer.h"
#include "onboard/perception/segmentation/leaf_proposer.h"
#include "onboard/perception/segmentation/ll_net_proposer.h"
#include "onboard/perception/segmentation/ouster_noise_proposer.h"
#include "onboard/perception/segmentation/proto/segmenter_config.pb.h"
#include "onboard/perception/segmentation/reflection_proposer.h"
#include "onboard/perception/segmentation/semantic_map_proposer.h"
#include "onboard/perception/segmentation/splash_proposer.h"

namespace qcraft::segmentation {

namespace {

std::unique_ptr<Proposer> GenerateProposer(const ProposerType type,
                                           ThreadPool* thread_pool) {
  switch (type) {
    case PT_FIERY_EYE_NET:
      return std::make_unique<FieryEyeNetProposer>(type, thread_pool);
    case PT_LL_NET:
      return std::make_unique<LLNetProposer>(type, thread_pool);
    case PT_OUSTER_NOISE:
      return std::make_unique<OusterNoiseProposer>(type, thread_pool);
    case PT_CATEYE:
      return std::make_unique<CateyeProposer>(type, thread_pool);
    case PT_BLOOMING:
      return std::make_unique<BloomingProposer>(type, thread_pool);
    case PT_SEMANTIC_MAP:
      return std::make_unique<SemanticMapProposer>(type, thread_pool);
    case PT_BARRIER:
      return std::make_unique<BarrierProposer>(type, thread_pool);
    case PT_LARGE_VEHICLE:
      return std::make_unique<LargeVehicleProposer>(type, thread_pool);
    case PT_LEAF:
      return std::make_unique<LeafProposer>(type, thread_pool);
    case PT_REFLECTION:
      return std::make_unique<ReflectionProposer>(type, thread_pool);
    case PT_SPLASH:
      return std::make_unique<SplashProposer>(type, thread_pool);
    case PT_BPEARL_NOISE:
      return std::make_unique<BpearlNoiseProposer>(type, thread_pool);
    case PT_DISTANT_CLUSTER:
      return std::make_unique<DistantClusterProposer>(type, thread_pool);
    case PT_NONE:
      QLOG(FATAL) << "Should not reach here. type: " << ProposerType_Name(type);
  }
}

TreeNodesProto LoadProposerTreeConfig(const std::string& conf_path) {
  TreeNodesProto tree_nodes_proto;
  QCHECK(file_util::FileToProto(conf_path, &tree_nodes_proto));
  return tree_nodes_proto;
}

std::vector<TreeNodeProto> GetTreeNodeChildren(
    const TreeNodesProto& tree_nodes_proto, const std::string& parent) {
  std::vector<TreeNodeProto> children;
  for (const auto& node : tree_nodes_proto.tree_nodes()) {
    if (node.parent() == parent) {
      children.push_back(node);
    }
  }
  return children;
}

void BuildProposerTree(const TreeNodesProto& tree_nodes_proto,
                       ProposerTreeNode* parent, const ProposerType type,
                       const int score, ThreadPool* thread_pool,
                       ProposerTree* proposer_tree) {
  QCHECK_NE(type, PT_NONE);
  QCHECK_NOTNULL(parent);
  QCHECK_NOTNULL(proposer_tree);

  auto* node = proposer_tree->AddChildToNode(
      parent, type, score, GenerateProposer(type, thread_pool));

  std::vector<TreeNodeProto> children =
      GetTreeNodeChildren(tree_nodes_proto, ProposerType_Name(type));
  for (const auto& child : children) {
    ProposerType child_type;
    QCHECK(ProposerType_Parse(child.name(), &child_type)) << child.name();
    BuildProposerTree(tree_nodes_proto, node, child_type, child.score(),
                      thread_pool, proposer_tree);
  }
}

ProposerTree BuildProposerTree(ThreadPool* thread_pool,
                               const TreeNodesProto& tree_nodes_proto) {
  std::vector<TreeNodeProto> root_node_children =
      GetTreeNodeChildren(tree_nodes_proto, "PT_NONE");
  QCHECK(!root_node_children.empty());

  ProposerTree proposer_tree(thread_pool);
  auto* root_node = proposer_tree.InitRootNode();
  for (const auto& child : root_node_children) {
    ProposerType child_type;
    QCHECK(ProposerType_Parse(child.name(), &child_type));
    BuildProposerTree(tree_nodes_proto, root_node, child_type, child.score(),
                      thread_pool, &proposer_tree);
  }
  proposer_tree.InitComplete();

  return proposer_tree;
}

}  // namespace

ProposerTree BuildProposerTree(ThreadPool* thread_pool,
                               const std::string& conf_path) {
  return BuildProposerTree(thread_pool, LoadProposerTreeConfig(conf_path));
}

}  // namespace qcraft::segmentation
