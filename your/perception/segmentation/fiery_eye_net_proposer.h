#ifndef ONBOARD_PERCEPTION_SEGMENTATION_FIERY_EYE_NET_PROPOSER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_FIERY_EYE_NET_PROPOSER_H_

#include "onboard/perception/segmentation/proposer.h"

namespace qcraft::segmentation {

class FieryEyeNetProposer final : public Proposer {
 public:
  using Proposer::Proposer;

 private:
  ProposedClusters ProposeInternal(const ProposerEnvInfo &env_info,
                                   const ProposedClusters &clusters) override;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_FIERY_EYE_NET_PROPOSER_H_
