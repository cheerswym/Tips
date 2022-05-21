#ifndef ONBOARD_PERCEPTION_SEGMENTATION_LEAF_PROPOSER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_LEAF_PROPOSER_H_

#include <vector>

#include "onboard/perception/segmentation/proposer.h"

namespace qcraft::segmentation {

struct LeafClusterInfo {
  Polygon2d contour;
  float delta_z = 0.f;
  float avg_clearance = 0.f;
};

class LeafProposer final : public Proposer {
 public:
  using Proposer::Proposer;

 private:
  ProposedClusters ProposeInternal(const ProposerEnvInfo& env_info,
                                   const ProposedClusters& clusters) override;

 private:
  std::vector<LeafClusterInfo> prev_leaf_clusters_info_;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_LEAF_PROPOSER_H_
