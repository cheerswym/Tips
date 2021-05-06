#ifndef ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_
#define ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_

#include <string>
#include <vector>

#include "absl/types/span.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/scheduler/scheduler_output.h"
// #include "onboard/planner/trajectory.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_validation_error.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

bool ValidatePlanStartPointCollisionFree(
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const PlannerObjectManager &obj_mgr, std::string *ptr_obj_id);

bool ValidateEstTrajectory(
    const SemanticMapManager &semantic_map_manager,
    const std::vector<PartialSpacetimeObjectTrajectory> &considered_st_objects,
    bool full_stop, const PoseProto &pose,
    const SchedulerOutput &scheduler_output,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto *result, ThreadPool *thread_pool);

bool ValidateEstPrevTrajectory(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    absl::Span<const ApolloTrajectoryPointProto> traj,
    TrajectoryValidationResultProto *result);

// Basic checks.
bool ValidateTrajectory(
    const PoseProto &current_pose,
    absl::Span<const TrajectoryPoint> traj_points,
    const TrajectoryValidationOptionsProto &trajectory_validation_options,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    TrajectoryValidationResultProto *result);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TRAJECTORY_VALIDATION_H_
