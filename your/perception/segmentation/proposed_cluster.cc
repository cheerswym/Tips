#include "onboard/perception/segmentation/proposed_cluster.h"

namespace qcraft::segmentation {
// NOTE(dong): This is the only interface used to initialize proposed clusters.
// It's not allowed to be called elsewhere unless segmenter and unit test.
ProposedClusters InitProposedClusters(ClusterVector raw_clusters) {
  ProposedClusters clusters;
  clusters.reserve(raw_clusters.size());
  for (auto& raw_cluster : raw_clusters) {
    clusters.emplace_back(ProposedCluster(std::move(raw_cluster)));
  }
  return clusters;
}

}  // namespace qcraft::segmentation
