#ifndef ONBOARD_PLANNER_EMERGENCY_STOP_H_
#define ONBOARD_PLANNER_EMERGENCY_STOP_H_

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace planner {
namespace aeb {

struct EmergencyStopInfo {
  std::string id;  // object id
  qcraft::ObjectType type;
};

// NOTE: If returns InternalError or PermissionDeniedError, risk_area remains
//  unmodified.
// TODO(weijun): Refactor this function by creating another function to compute
// risk area.
absl::StatusOr<EmergencyStopInfo> CheckEmergencyStopByCircularMotion(
    const VehicleGeometryParamsProto &vehicle_geom,
    const EmergencyStopParamsProto &emergency_stop_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const PoseProto &vehicle_pose, const ObjectsProto &objects_proto,
    const Chassis &chassis, Polygon2d *output_risk_area);

std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto &plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto> &prev_traj_points,
    const EmergencyStopParamsProto &emergency_stop_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const SemanticMapManager &semantic_map_manager);

std::shared_ptr<const ObjectsProto> ExportObjectsForAEB(
    const std::shared_ptr<const ObjectsProto> &real_objects,
    const std::shared_ptr<const ObjectsProto> &virtual_objects);

absl::StatusOr<EmergencyStopInfo> CheckEmergencyStop(
    const PlannerParams &planner_params, const PlannerInput &input,
    Polygon2d *output_risk_area);

std::vector<ApolloTrajectoryPointProto> GenerateStopTrajectory(
    double init_s, bool reset, bool forward, double max_deceleration,
    const MotionConstraintParamsProto &motion_constraint_params,
    const TrajectoryPoint &plan_start_traj_point,
    const std::vector<ApolloTrajectoryPointProto> &prev_trajectory);

}  // namespace aeb
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_EMERGENCY_STOP_H_
