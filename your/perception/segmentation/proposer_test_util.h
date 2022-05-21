#ifndef ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TEST_UTIL_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TEST_UTIL_H_

#include <optional>

#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/vis/common/color.h"

namespace qcraft::segmentation {

void DrawProposedClusterToCanvas(
    const ProposedCluster& cluster, double z_offset,
    const std::optional<vis::Color>& color = std::nullopt);

inline void DrawProposedClustersToCanvas(
    const ProposedClusters& clusters, double z_offset,
    const std::optional<vis::Color>& color = std::nullopt) {
  for (const auto& cluster : clusters) {
    DrawProposedClusterToCanvas(cluster, z_offset, color);
  }
}

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_PROPOSER_TEST_UTIL_H_
