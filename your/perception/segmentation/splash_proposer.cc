#include "onboard/perception/segmentation/splash_proposer.h"

namespace qcraft::segmentation {

namespace {}

ProposedClusters SplashProposer::ProposeInternal(
    const ProposerEnvInfo& env_info, const ProposedClusters& clusters) {
  return clusters;
}
}  // namespace qcraft::segmentation
