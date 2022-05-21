#include "onboard/perception/segmentation/pyramid.h"

#include <functional>
#include <utility>

namespace qcraft::segmentation {

void Pyramid::BuildClusterPyramid(const SceneMap& scene_map,
                                  const ProposedClusterDag& dag) {
  SCOPED_QTRACE("BuildClusterPyramid");
  std::function<void(ProposerTreeNodePtr)> dfs = [&](ProposerTreeNodePtr node) {
    const int depth = node->depth();
    const auto type = node->type();
    const auto& children = node->children();
    // Init clusters for leaf proposers.
    if (children.empty()) {
      const auto& clusters = scene_map.GetResult(type);
      ProposedClusterPtrs cluster_ptrs;
      cluster_ptrs.reserve(clusters.size());
      for (const auto& cluster : clusters) {
        cluster_ptrs.emplace_back(&cluster);
      }
      UpdateWithSelectedClusters(depth, type, std::move(cluster_ptrs));
      return;
    }

    for (const auto& child : children) {
      dfs(child.get());
    }
    // If a parent proposer only has one child. No need to select.
    if (children.size() == 1) {
      UpdateWithSelectedClusters(
          depth, type, GetClusters(children[0]->depth(), children[0]->type()));
      return;
    }
    // Pack and select.
    ProposedClusterPacks packs;
    for (const auto& child : children) {
      const auto bigraph = dag.ComputeBigraph(
          type, child->type(), GetClusters(child->depth(), child->type()));
      auto sub_packs = GroupPacks(bigraph);

      QCHECK_EQ(CalculateTotalNumParentClusters(sub_packs),
                scene_map.GetResult(type).size());

      packs.insert(packs.end(), std::make_move_iterator(sub_packs.begin()),
                   std::make_move_iterator(sub_packs.end()));
    }
    UpdateWithSelectedClusters(depth, type,
                               SelectClustersFromPacks(std::move(packs)));
  };

  dfs(proposer_tree_.root_node());
}

}  // namespace qcraft::segmentation
