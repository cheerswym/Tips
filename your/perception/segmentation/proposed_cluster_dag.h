#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_DAG_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_DAG_H_

#include <map>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/scene_map.h"
#include "onboard/perception/segmentation/types.h"

namespace qcraft::segmentation {
// NOTE(dong): Each pack is a connected component in the bigraph.
struct ProposedClusterPack {
  ProposedClusterPtrs parent_clusters;
  ProposedClusterPtrs child_clusters;
  double weighted_score = 0.0;
};
using ProposedClusterPacks = std::vector<ProposedClusterPack>;

// Accumulate num parent clusters from packs.
int CalculateTotalNumParentClusters(const ProposedClusterPacks& packs);
// Accumulate num child clusters from packs.
int CalculateTotalNumChildClusters(const ProposedClusterPacks& packs);
// NOTE(dong): Select clusters according to the weighted score of pack.
ProposedClusterPtrs SelectClustersFromPacks(ProposedClusterPacks packs);

struct ProposedClusterStage {
  ProposerType type;
  ProposedClusterPtrs clusters;
};
using ProposedClusterHistory = std::vector<ProposedClusterStage>;

using ProposedClusterOneToMultiMap =
    absl::flat_hash_map<ProposedClusterPtr, ProposedClusterPtrs>;
// NOTE(dong): A bipartite graph (or bigraph) is a graph whose vertices can be
// divided into two disjoint and independent sets U and V such that every edge
// connects a vertex in U to one in V. In our design, U is the set of
// parent/child clusters while V is the set of child/parent clusters.
struct ProposedClustersBigraph {
  ProposerType parent_type;
  ProposerType child_type;
  ProposedClusterOneToMultiMap parent_to_childs_map;
  ProposedClusterOneToMultiMap child_to_parents_map;
};
// NOTE(dong): Collect all connected components in the bigraph.
ProposedClusterPacks GroupPacks(const ProposedClustersBigraph& bigraph);
// NOTE(dong): Here we introduce a directed acyclic graph (DAG) to represent
// dependencies between proposed clusters from different proposers.
// Relations between parent clusters and child clusters are constructed
// using one to multi maps.
class ProposedClusterDag {
  using ObstacleToClusterMap =
      absl::flat_hash_map<ObstaclePtr, ProposedClusterPtr>;

 public:
  explicit ProposedClusterDag(const SceneMap& scene_map)
      : scene_map_(scene_map) {
    InitObstacleToClusterMap();
  }
  // NOTE(dong): Compute the bigraph of parent clusters and child clusters.
  // Parent clusters can be found in the scene map and so don't need to be
  // input. Child clusters are selected results and usually can't be found in
  // the scene map.
  ProposedClustersBigraph ComputeBigraph(
      const ProposerType parent_type, const ProposerType child_type,
      const ProposedClusterPtrs& child_clusters) const;

  ProposedClusterHistory GetProposedClusterHistory(
      const ProposedCluster& cluster) const;

 private:
  void InitObstacleToClusterMap();

 private:
  const SceneMap& scene_map_;
  std::map<ProposerType, ObstacleToClusterMap> obstacle_to_cluster_maps_;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSED_CLUSTER_DAG_H_
