#ifndef ONBOARD_PERCEPTION_SEGMENTATION_SPLASH_PROPOSER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_SPLASH_PROPOSER_H_

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/proposer.h"

namespace qcraft::segmentation {

class SplashProposer final : public Proposer {
 public:
  using Proposer::Proposer;

 private:
  ProposedClusters ProposeInternal(const ProposerEnvInfo& env_info,
                                   const ProposedClusters& clusters) override;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_SPLASH_PROPOSER_H_
