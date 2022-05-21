#ifndef ONBOARD_PERCEPTION_GROUND_REMOVAL_GRADIENT_BASED_GROUND_REMOVER_H_
#define ONBOARD_PERCEPTION_GROUND_REMOVAL_GRADIENT_BASED_GROUND_REMOVER_H_

#include "onboard/perception/ground_removal/ground_remover.h"

namespace qcraft::ground_removal {

class GradientBasedGroundRemover final : public GroundRemover {
 public:
  using GroundRemover::GroundRemover;

  void Compute() override;
};

}  // namespace qcraft::ground_removal

#endif  // ONBOARD_PERCEPTION_GROUND_REMOVAL_GRADIENT_BASED_GROUND_REMOVER_H_
