#ifndef ONBOARD_PLANNER_SPEED_CONSTRAINT_MANAGER_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_CONSTRAINT_MANAGER_DECIDER_H_

#include <vector>

#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"

namespace qcraft {
namespace planner {

void MakeConstraintDecisionForStBoundaries(
    const ConstraintManager& constraint_mgr,
    std::vector<StBoundaryWithDecision>* st_boundaries_wd);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_CONSTRAINT_MANAGER_DECIDER_H_
