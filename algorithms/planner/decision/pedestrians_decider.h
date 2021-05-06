#ifndef ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_PEDESTRIANS_DECIDER_H_

#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildPedestriansConstraints(
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerSemanticMapManager &psmm,
    const ApolloTrajectoryPointProto &plan_start_point,
    const DrivePassage &passage, const PathSlBoundary &sl_boundary,
    const SpacetimeTrajectoryManager &st_traj_mgr);

}  // namespace planner
}  // namespace qcraft

#endif
