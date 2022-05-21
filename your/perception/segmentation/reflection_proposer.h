#ifndef ONBOARD_PERCEPTION_SEGMENTATION_REFLECTION_PROPOSER_H_
#define ONBOARD_PERCEPTION_SEGMENTATION_REFLECTION_PROPOSER_H_

#include <utility>
#include <vector>

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/perception/obstacle_manager.h"
#include "onboard/perception/segmentation/proposed_cluster.h"
#include "onboard/perception/segmentation/proposer.h"

namespace qcraft::segmentation {

class ReflectionProposer final : public Proposer {
 public:
  using Proposer::Proposer;

 private:
  ProposedClusters ProposeInternal(const ProposerEnvInfo& env_info,
                                   const ProposedClusters& clusters) override;

 private:
  std::vector<std::pair<double, Box2d>> sign_zones_history_;
};

}  // namespace qcraft::segmentation

#endif  // ONBOARD_PERCEPTION_SEGMENTATION_REFLECTION_PROPOSER_H_
