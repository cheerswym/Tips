#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PYRAMID_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PYRAMID_H_

#include <map>
#include <optional>
#include <utility>

#include "onboard/perception/segmentation/proposed_cluster_dag.h"
#include "onboard/perception/segmentation/proposer_tree.h"
#include "onboard/perception/segmentation/scene_map.h"

namespace qcraft::segmentation {
// NOTE(dong): Pyramid is a different form to represent the proposer tree. Users
// can find child proposer types of a parent proposer or selected proposed
// clusters of a certain proposer in the pyramid. Selected clusters of the
// bottom layer are directly constructed from the scene map. After that,
// each upper layer will be updated by the selection results of the previous
// layer.
class Pyramid {
 public:
  using ClusterPyramid =
      std::map<int, std::map<ProposerType, std::optional<ProposedClusterPtrs>>>;

 public:
  explicit Pyramid(const ProposerTree& proposer_tree)
      : proposer_tree_(proposer_tree) {
    auto init_pyramid = [&](ProposerTreeNodePtr node) {
      InsertOrDie(&cluster_pyramid_[node->depth()], node->type(), {});
    };
    proposer_tree.Traverse(std::move(init_pyramid));
  }
  // NOTE(dong): Select and update clusters in the pyramid. Clusters of each
  // proposer are selected from clusters of all its child proposers, except leaf
  // proposers. Leaf proposers construct their clusters directly from the scene
  // map.
  void BuildClusterPyramid(const SceneMap& scene_map,
                           const ProposedClusterDag& dag);

  const ClusterPyramid& cluster_pyramid() const { return cluster_pyramid_; }

  const ProposedClusterPtrs& GetClusters(const int depth,
                                         const ProposerType type) const {
    const auto& clusters = FindOrDie(FindOrDie(cluster_pyramid_, depth), type);
    QCHECK(clusters) << absl::StrFormat(": depth %d, %s", depth,
                                        ProposerType_Name(type));
    return *clusters;
  }

 private:
  void UpdateWithSelectedClusters(const int depth, const ProposerType type,
                                  ProposedClusterPtrs proposed_clusters) {
    auto& clusters = FindOrDie(FindOrDie(cluster_pyramid_, depth), type);
    QCHECK(!clusters) << absl::StrFormat(": depth %d, %s", depth,
                                         ProposerType_Name(type));
    clusters = std::move(proposed_clusters);
  }

 private:
  const ProposerTree& proposer_tree_;
  ClusterPyramid cluster_pyramid_;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PYRAMID_H_
