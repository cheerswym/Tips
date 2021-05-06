#ifndef ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_
#define ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_

#include <utility>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"

namespace qcraft::planner {

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromDrivePassage(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage);

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geom,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const LaneChangeStateProto &lc_state,
    const SmoothedReferenceLineResultMap &smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth_next_left_turn);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_PATH_BOUNDARY_BUILDER_H_
