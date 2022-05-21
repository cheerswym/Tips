#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TREE_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TREE_H_

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/async/async_util.h"
#include "onboard/async/thread_pool.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/proposer.h"
#include "onboard/perception/segmentation/proto/segmenter_config.pb.h"
#include "onboard/perception/segmentation/scene_map.h"
#include "onboard/perception/segmentation/types.h"

namespace qcraft::segmentation {

class ProposerTreeNode;  // forward declaration
using ProposerTreeNodeRef = std::unique_ptr<ProposerTreeNode>;
using ProposerTreeNodePtr = const ProposerTreeNode*;
using ProposerTreeNodePtrs = std::vector<ProposerTreeNodePtr>;

// A ProposerTreeNode indicates a certain proposer's attributes and
// bindings in the proposer tree.
//
// [Attributes]: type, score, depth and score of depth.
//
// 1. type
// Each node has a unique type and a corresponding proposer function.
// 2. score
// Each node has an initial score ranging in [1,9]. The higher the score, the
// more likely it is to be selected. The score of the root node is 0.
// 3. depth
// Depth in proposer tree.
// 4. score of depth
// The weighted score calculates from the node's score and depth. The deeper the
// proposer goes, the smaller its weighted score will be. The proposer score of
// a proposed cluster is the sum of all its proposers' weighted scores.
//
// [Bindings]: parent and children.
class ProposerTreeNode {
 public:
  ProposerTreeNode()
      : parent_(nullptr),
        type_(PT_NONE),
        score_(0),
        depth_(0),
        score_of_depth_(0.0f) {}

  ProposerTreeNode(ProposerTreeNodePtr parent, const ProposerType type,
                   const int score, ProposerRef proposer);

  ProposerTreeNode* AddChild(const ProposerType type, const int score,
                             ProposerRef proposer) {
    children_.emplace_back(std::make_unique<ProposerTreeNode>(
        this, type, score, std::move(proposer)));
    return children_.back().get();
  }

  ProposerTreeNode* AddChild(ProposerTreeNodeRef node) {
    children_.emplace_back(std::move(node));
    return children_.back().get();
  }

  ProposerType type() const { return type_; }
  int score() const { return score_; }
  int depth() const { return depth_; }
  double score_of_depth() const { return score_of_depth_; }

  const std::vector<ProposerTreeNodeRef>& children() const { return children_; }
  ProposerTreeNodePtr parent() const { return parent_; }
  // NOTE(dong): update proposed cluster score after propose.
  ProposedClusters ProposeAndUpdate(const ProposerEnvInfo& env_info,
                                    const ProposedClusters& clusters) const {
    auto proposed_clusters = proposer_->Propose(env_info, clusters);
    for (auto& proposed_cluster : proposed_clusters) {
      if (proposed_cluster.is_proposed()) {
        proposed_cluster.AppendProposerHistory(type_);
        UpdateProposerScore(&proposed_cluster);
      }
      proposed_cluster.set_is_proposed(false);
    }
    return proposed_clusters;
  }

 private:
  // NOTE(dong): Each increase in depth reduce the score to one-tenth of its
  // previous value.
  double CalculateScoreOfDepth(const int score, const int depth_) const {
    QCHECK_GE(depth_, 0);
    QCHECK_GE(score, 1);
    QCHECK_LE(score, 9);

    return score * std::pow(0.1, depth_);
  }
  void UpdateProposerScore(ProposedCluster* cluster) const {
    cluster->AddProposerScore(score_of_depth_);
  }

 private:
  // node attribute
  ProposerTreeNodePtr parent_;
  const ProposerType type_;
  const int score_;
  const int depth_;
  const double score_of_depth_;
  std::vector<ProposerTreeNodeRef> children_;
  // proposer
  ProposerRef proposer_;
};

using ProposerTreeNodeMap =
    absl::flat_hash_map<ProposerType, ProposerTreeNodePtr>;

class ProposerTree {
 public:
  explicit ProposerTree(ThreadPool* const thread_pool)
      : thread_pool_(thread_pool) {}
  ProposerTreeNode* InitRootNode();
  ProposerTreeNode* AddChildToNode(ProposerTreeNode* parent,
                                   const ProposerType type, const int score,
                                   ProposerRef proposer);
  // Breadth first traversal
  SceneMap TraverseAndPropose(const ProposerEnvInfo&, ProposedClusters);
  // A recursive function to traverse the given tree node.
  void TraverseAndPropose(const ProposerEnvInfo& env_info,
                          const ProposedClusters& parent_result,
                          ProposerTreeNodePtr node, SceneMap* scene_map) const;
  // Breadth first traversal
  void Traverse(std::function<void(ProposerTreeNodePtr)> func) const;
  //
  TreeNodesProto ToProto() const;

  bool initialized() const { return initialized_; }
  void InitComplete() { initialized_ = true; }

  ProposerTreeNodePtr root_node() const { return root_node_.get(); }

 private:
  void UpdateNodeMap(const ProposerType type, ProposerTreeNodePtr node);

 private:
  bool initialized_ = false;
  ThreadPool* const thread_pool_;
  ProposerTreeNodeRef root_node_;
  // for quick index
  ProposerTreeNodeMap node_map_;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TREE_H_
