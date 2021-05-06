#ifndef ONBOARD_PLANNER_DECISION_CAUTIOUS_BRAKE_H_
#define ONBOARD_PLANNER_DECISION_CAUTIOUS_BRAKE_H_

#include <vector>

#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
namespace qcraft {
namespace planner {

// This function return cautious brake constraint.
std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager &planner_semantic_map_manager,
    const DrivePassage &passage, const SpacetimeTrajectoryManager &st_traj_mgr);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_CAUTIOUS_BRAKE_H_
