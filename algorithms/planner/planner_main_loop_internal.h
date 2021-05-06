#ifndef ONBOARD_PLANNER_PLANNER_MAIN_LOOP_INTERNAL_H_
#define ONBOARD_PLANNER_PLANNER_MAIN_LOOP_INTERNAL_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_join.h"
#include "absl/time/time.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route_manager_output.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/remote_assist.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

void RecoverPlannerStateFromProto(
    const PlannerStateProto &state_proto,
    const PlannerSemanticMapManager &planner_semantic_map_manager,
    const RouteManagerOutputProto &route_output_proto,
    const VehicleGeometryParamsProto &vehicle_geom, PlannerState *state);

std::shared_ptr<const ObjectsProto> GetAllObjects(
    std::shared_ptr<const ObjectsProto> real_objects,
    std::shared_ptr<const ObjectsProto> virtual_objects);

std::optional<int> InterpolatePointFromPrevTrajectory(
    absl::Time time, const TrajectoryProto &prev_traj);

bool InterpolatePointFromPrevTrajectoryIncludingPast(
    absl::Time time, const TrajectoryProto &prev_traj,
    ApolloTrajectoryPointProto *point);

bool MaybeResetEstPlanner(
    const ApolloTrajectoryPointProto &pre_reset_planned_point,
    Vec2d current_pos, std::vector<std::string> *reset_reasons);

void FillTrajectoryProto(
    absl::Time plan_time, const RouteManagerOutput &route_output,
    const std::vector<ApolloTrajectoryPointProto> &planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto> &past_points,
    const mapping::LanePath &target_lane_path_from_current,
    const LaneChangeStateProto &lane_change_state, TurnSignal turn_signal,
    const DoorDecision &door_decision, bool is_aeb_triggered,
    const DrivingStateProto &driving_state,
    const TrajectoryValidationResultProto &validate_result,
    TrajectoryProto *trajectory);

void ConvertTrajectoryToGlobalCoordinates(
    const CoordinateConverter &coordinate_converter,
    const TrajectoryProto &trajectory,
    std::vector<PlannerState::PosePoint> *previous_trajectory_global,
    std::vector<PlannerState::PosePoint> *previous_past_trajectory_global);

void ReportCandidateTrafficLightInfo(
    const TrafficLightInfoMap &traffic_light_map, PlannerDebugProto *debug);

void ReportSelectedTrafficLightInfo(
    const TrafficLightInfoMap &traffic_light_map,
    const mapping::LanePath &lane_path, TrafficLightInfoProto *proto);
// Prev traj includes past points. Prev traj will never be empty once the
// first successful planner iteration completes.
std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto &previous_trajectory,
    const MotionConstraintParamsProto &motion_constraint_params);

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> FallBackToPreviousPlan(
    const std::vector<ApolloTrajectoryPointProto> &time_aligned_prev_traj,
    bool reset);

bool NeedForceResetEstPlanner(const TrajectoryProto &prev_trajectory,
                              const AutonomyStateProto &now_autonomy_state,
                              const AutonomyStateProto &prev_autonomy_state,
                              bool previously_triggered_aeb,
                              bool is_emergency_stop, bool rerouted,
                              bool full_stopped,
                              std::vector<std::string> *reset_reasons);

// Export data from planner to HMI content.
void ReportHmiContent(const RouteContentProto &route_content,
                      const SemanticMapManager &semantic_map_manager,
                      const mapping::LanePath &lane_path, HmiContentProto *hmi);

void ReportTeleopStatus(const TeleopState &teleop_state,
                        const SemanticMapManager &semantic_map_manager,
                        const CoordinateConverter &coordinate_converter,
                        PlannerTeleopStatusProto *teleop_status);

absl::StatusOr<mapping::LanePath> FindRouteLanePathFromStartPoint(
    const PlannerSemanticMapManager &psmm,
    const mapping::LanePath &raw_lane_path, double projection_range,
    const ApolloTrajectoryPointProto &start_point, double *travel_arc_len);

PlanStartPointInfo ComputeEstPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto &prev_trajectory,
    const PoseProto &pose, const AutonomyStateProto &now_autonomy_state,
    const AutonomyStateProto &prev_autonomy_state,
    bool previously_triggered_aeb, bool rerouted, bool aeb,
    const Chassis &chassis,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &vehicle_geom_params,
    const VehicleDriveParamsProto &vehicle_drive_params);

absl::Duration GetStPathPlanLookAheadTime(
    const PlanStartPointInfo &plan_start_point_info, const PoseProto &pose,
    const TrajectoryProto &previous_trajectory);

StPathPlanStartPointInfo GetStPathPlanStartPointInfo(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo &plan_start_point_info,
    const TrajectoryProto &previous_trajectory);

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &route_sections_from_start,
    const absl::flat_hash_set<mapping::ElementId> &avoid_lanes, double ego_v,
    const PointOnRouteSections &ego_proj,
    const LaneChangeRequestProto &lc_request);

// Rewrite trajectory_point and past_points with stationary trajectory points
// according current pose. The rest of the TrajectoryProto info remains
// unchanged.
absl::Status TranslateToStationaryTrajectory(const PoseProto &pose,
                                             TrajectoryProto *trajectory);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_MAIN_LOOP_INTERNAL_H_
