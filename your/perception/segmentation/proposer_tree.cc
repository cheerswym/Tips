#include "onboard/perception/segmentation/proposer_tree.h"

#include <memory>
#include <queue>
#include <utility>

#include "onboard/async/parallel_for.h"

namespace qcraft::segmentation {

ProposerTreeNode::ProposerTreeNode(ProposerTreeNodePtr parent,
                                   const ProposerType type, const int score,
                                   ProposerRef proposer)
    : parent_(QCHECK_NOTNULL(parent)),
      type_(type),
      score_(score),
      depth_(parent_->depth_ + 1),
      score_of_depth_(CalculateScoreOfDepth(score_, depth_)),
      proposer_(std::move(proposer)) {}

ProposerTreeNode* ProposerTree::InitRootNode() {
  root_node_ = std::make_unique<ProposerTreeNode>();
  UpdateNodeMap(root_node_->type(), root_node_.get());

  return root_node_.get();
}

ProposerTreeNode* ProposerTree::AddChildToNode(ProposerTreeNode* parent,
                                               const ProposerType type,
                                               const int score,
                                               ProposerRef proposer) {
  auto* child = parent->AddChild(type, score, std::move(proposer));
  UpdateNodeMap(child->type(), child);

  return child;
}

void ProposerTree::UpdateNodeMap(const ProposerType type,
                                 ProposerTreeNodePtr node) {
  InsertOrDie(&node_map_, type, node);
}

void ProposerTree::TraverseAndPropose(const ProposerEnvInfo& env_info,
                                      const ProposedClusters& parent_result,
                                      ProposerTreeNodePtr node,
                                      SceneMap* scene_map) const {
  const int num_children = node->children().size();
  std::vector<ProposedClusters> proposed_clusters(num_children);
  ParallelFor(0, num_children, thread_pool_, [&](int i) {
    proposed_clusters[i] =
        node->children()[i]->ProposeAndUpdate(env_info, parent_result);
  });
  for (int i = 0; i < num_children; ++i) {
    TraverseAndPropose(env_info, proposed_clusters[i],
                       node->children()[i].get(), scene_map);
    scene_map->emplace(node->children()[i]->type(),
                       std::move(proposed_clusters[i]));
  }
}

// NOTE(dong): Here we use BFS instead of DFS so that proposers in the same
// layer are able to run in parallel.
SceneMap ProposerTree::TraverseAndPropose(const ProposerEnvInfo& env_info,
                                          ProposedClusters root_clusters) {
  QCHECK(initialized());

  SceneMap scene_map;
  TraverseAndPropose(env_info, root_clusters, root_node_.get(), &scene_map);
  scene_map.emplace(root_node_->type(), std::move(root_clusters));
  return scene_map;
}

void ProposerTree::Traverse(
    std::function<void(ProposerTreeNodePtr)> func) const {
  QCHECK(initialized());
  std::queue<ProposerTreeNodePtr> node_queue;
  node_queue.push(root_node_.get());
  while (!node_queue.empty()) {
    const auto* node = node_queue.front();
    node_queue.pop();
    func(node);
    for (const auto& child : node->children()) {
      node_queue.emplace(child.get());
    }
  }
}

TreeNodesProto ProposerTree::ToProto() const {
  QCHECK(initialized());
  TreeNodesProto tree_nodes_proto;
  std::queue<ProposerTreeNodePtr> node_queue;
  for (const auto& child : root_node_->children()) {
    node_queue.emplace(child.get());
  }
  while (!node_queue.empty()) {
    const auto* node = node_queue.front();
    node_queue.pop();
    auto* tree_node_proto = tree_nodes_proto.add_tree_nodes();
    tree_node_proto->set_name(ProposerType_Name(node->type()));
    tree_node_proto->set_parent(ProposerType_Name(node->parent()->type()));
    tree_node_proto->set_score(node->score());
    for (const auto& child : node->children()) {
      node_queue.emplace(child.get());
    }
  }
  return tree_nodes_proto;
}

}  // namespace qcraft::segmentation
