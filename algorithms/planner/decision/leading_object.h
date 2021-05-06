#ifndef ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
#define ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/planner.pb.h"
namespace qcraft {
namespace planner {

// Returns leading objects that we should not pass. Currently all leading
// objects are associated with our lane path.
std::vector<ConstraintProto::LeadingObjectProto> FindLeadingObjects(
    const qcraft::VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerSemanticMapManager &psmm,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const SceneOutputProto &scene_reasoning, const DrivePassage &passage,
    const PathSlBoundary &sl_boundary,
    const std::optional<ClearanceCheckOutput> &lc_clearance_check_output,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const ApolloTrajectoryPointProto &plan_start_point,
    const FrenetBox &av_frenet_box, bool borrow_lane_boundary);

std::vector<ConstraintProto::LeadingObjectProto> ConvertLaneChangeTargets(
    const DrivePassage &passage, const SpacetimeTrajectoryManager &st_traj_mgr,
    const std::vector<std::string> &lc_targets);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_LEADING_OBJECT_H_
