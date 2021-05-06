#ifndef ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_

#include <vector>

#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"

namespace qcraft {
namespace planner {
// For freespace planner.
// TODO(ping): Refactor the code to make all speed finders share a common
// st-boundary pre-decider.
struct FreespaceStBoundaryPreDecisionInput {
  // Av inputs.
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  // Param inputs.
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
};

struct FreespaceStBoundaryPreDecisionOutput {
  // Boundaries with decisions.
  std::vector<StBoundaryWithDecision> st_boundaries_with_decision;
};

void MakeFreespaceStBoundaryPreDecision(
    const FreespaceStBoundaryPreDecisionInput& input,
    std::vector<StBoundaryWithDecision>* st_boundaries_with_decision);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_ST_BOUNDARY_PRE_DECIDER_H_
