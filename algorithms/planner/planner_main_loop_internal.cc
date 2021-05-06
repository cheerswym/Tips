#include "onboard/planner/planner_main_loop_internal.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/plan/plan_task_helper.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/proto/smooth_reference_line.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/utils/objects_view.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

bool MaybeReset(const ApolloTrajectoryPointProto &pre_reset_planned_point,
                Vec2d current_pos, double longitudinal_reset_error,
                double lateral_reset_error, const std::string &planner_name,
                std::vector<std::string> *reset_reasons) {
  const Vec2d pos_diff =
      current_pos - Vec2d(pre_reset_planned_point.path_point().x(),
                          pre_reset_planned_point.path_point().y());
  const Vec2d planned_tangent =
      Vec2d::FastUnitFromAngle(pre_reset_planned_point.path_point().theta());

  const double longitudinal_error = std::abs(pos_diff.Dot(planned_tangent));
  if (longitudinal_error > longitudinal_reset_error) {
    QEVENT("renjie", absl::StrCat("planner_reset_", planner_name, "_lon_error"),
           [longitudinal_error](QEvent *qevent) {
             qevent->AddField("lon_error", longitudinal_error);
           });
    QLOG(ERROR) << "Resetting due to longitudinal error.";
    reset_reasons->push_back(
        absl::StrCat(planner_name, " longitudinal error."));
    return true;
  }

  const double lateral_error = std::abs(pos_diff.Dot(planned_tangent.Perp()));
  if (lateral_error > lateral_reset_error) {
    QEVENT("renjie", absl::StrCat("planner_reset_", planner_name, "_lat_error"),
           [lateral_error](QEvent *qevent) {
             qevent->AddField("lat_error", lateral_error);
           });
    QLOG(ERROR) << "Resetting due to lateral error.";
    reset_reasons->push_back(absl::StrCat(planner_name, " lateral error."));
    return true;
  }

  return false;
}

}  // namespace

void RecoverPlannerStateFromProto(
    const PlannerStateProto &state_proto,
    const PlannerSemanticMapManager &planner_semantic_map_manager,
    const RouteManagerOutputProto &route_output_proto,
    const VehicleGeometryParamsProto &vehicle_geom, PlannerState *state) {
  const auto &semantic_map_manager =
      *planner_semantic_map_manager.semantic_map_manager();
  QCHECK_NOTNULL(state);
  state->FromProto(state_proto);
  bool can_upgrade = state->Upgrade();
  if (!can_upgrade) {
    VLOG(1) << "Cannot upgrade, old version: " << state->version;
  }
  state->planner_semantic_map_modifier = CreateSemanticMapModification(
      semantic_map_manager, state_proto.planner_semantic_map_modifier());
  state->prev_lane_path_before_lc.set_semantic_map_manager(
      &semantic_map_manager);
  state->prev_lane_path_before_lc.FromProto(
      state_proto.prev_lane_path_before_lc());
  state->prev_target_lane_path.set_semantic_map_manager(&semantic_map_manager);
  state->prev_target_lane_path.FromProto(state_proto.prev_target_lane_path());
  state->preferred_lane_path.set_semantic_map_manager(&semantic_map_manager);
  state->preferred_lane_path.FromProto(state_proto.preferred_lane_path());

  state->smooth_result_map.Clear();
  if (state_proto.has_smooth_result_map()) {
    for (const auto &lane_id_vec :
         state_proto.smooth_result_map().lane_id_vec()) {
      std::vector<mapping::ElementId> lane_ids;
      lane_ids.reserve(lane_id_vec.lane_id_size());
      for (const auto &lane_id : lane_id_vec.lane_id()) {
        lane_ids.push_back(lane_id);
      }
      const double half_av_width = vehicle_geom.width() * 0.5;
      const auto smoothed_result = SmoothLanePathByLaneIds(
          planner_semantic_map_manager, lane_ids, half_av_width);
      if (smoothed_result.ok()) {
        state->smooth_result_map.AddResult(lane_ids,
                                           std::move(smoothed_result).value());
      }
    }
  }

  // Backward compatibility: No plan task queue in earlier log.
  if (state_proto.plan_task_queue().empty()) {
    RouteManagerOutput route_output;
    route_output.FromProto(planner_semantic_map_manager.semantic_map_manager(),
                           route_output_proto);
    state->plan_task_queue = CreatePlanTasksQueueFromRoutingResult(
        route_output, *planner_semantic_map_manager.semantic_map_manager());
  }
}

std::shared_ptr<const ObjectsProto> GetAllObjects(
    std::shared_ptr<const ObjectsProto> real_objects,
    std::shared_ptr<const ObjectsProto> virtual_objects) {
  SCOPED_QTRACE("GetAllObjects");

  ObjectsView objects_view;
  if (real_objects != nullptr) {
    objects_view.UpdateObjects(ObjectsProto::SCOPE_REAL, real_objects);
  }
  if (virtual_objects != nullptr) {
    objects_view.UpdateObjects(ObjectsProto::SCOPE_VIRTUAL, virtual_objects);
  }
  return objects_view.ExportAllObjectsProto();
}

std::optional<int> InterpolatePointFromPrevTrajectory(
    absl::Time time, const TrajectoryProto &prev_traj) {
  const double t = ToUnixDoubleSeconds(time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      t > prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      t < prev_traj_start_time) {
    return std::nullopt;
  }

  // Previous trajectory should have equal time interval of kTrajectoryTimeStep.
  return RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
}

bool InterpolatePointFromPrevTrajectoryIncludingPast(
    absl::Time time, const TrajectoryProto &prev_traj,
    ApolloTrajectoryPointProto *point) {
  const double t = ToUnixDoubleSeconds(time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() || prev_traj.past_points().empty() ||
      t > prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      t < prev_traj_start_time +
              prev_traj.past_points().begin()->relative_time()) {
    return false;
  }

  // Both previous trajectory and previous past trajectory should have equal
  // time interval of kTrajectoryTimeStep.
  const int relative_time_index =
      RoundToInt((t - prev_traj_start_time) / kTrajectoryTimeStep);
  QCHECK_GE(relative_time_index, -prev_traj.past_points_size());
  QCHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
  *point = relative_time_index < 0
               ? prev_traj.past_points(relative_time_index +
                                       prev_traj.past_points_size())
               : prev_traj.trajectory_point(relative_time_index);

  return true;
}

bool MaybeResetEstPlanner(
    const ApolloTrajectoryPointProto &pre_reset_planned_point,
    Vec2d current_pos, std::vector<std::string> *reset_reasons) {
  constexpr double kLongitudinalErrorForReset = 3.0;  // m.
  constexpr double kLateralErrorForReset = 0.55;      // m.
  return MaybeReset(pre_reset_planned_point, current_pos,
                    kLongitudinalErrorForReset, kLateralErrorForReset,
                    "est_planner", reset_reasons);
}

void FillTrajectoryProto(
    absl::Time plan_time, const RouteManagerOutput &route_output,
    const std::vector<ApolloTrajectoryPointProto> &planned_trajectory,
    const std::vector<ApolloTrajectoryPointProto> &past_points,
    const mapping::LanePath &target_lane_path_from_current,
    const LaneChangeStateProto &lane_change_state, TurnSignal turn_signal,
    const DoorDecision &door_decision, bool is_aeb_triggered,
    const DrivingStateProto &driving_state,
    const TrajectoryValidationResultProto &validate_result,
    TrajectoryProto *trajectory) {
  SCOPED_QTRACE("FillTrajectoryProto");

  trajectory->set_trajectory_start_timestamp(ToUnixDoubleSeconds(plan_time));
  for (int i = 0; i < planned_trajectory.size(); ++i) {
    *trajectory->add_trajectory_point() = planned_trajectory[i];
  }

  // NOTE: past_points are designed for controller.
  for (const auto &past_point : past_points) {
    *trajectory->add_past_points() = past_point;
  }

  route_output.route->ToProto(trajectory->mutable_route_proto());

  target_lane_path_from_current.ToProto(
      trajectory->mutable_target_lane_path_from_current());

  trajectory->set_turn_signal(turn_signal);

  // TODO(renjie): redesign it after onboard freespace planner.
  trajectory->set_gear(Chassis::GEAR_DRIVE);

  *(trajectory->mutable_door_decision()) = door_decision;

  trajectory->set_aeb_triggered(is_aeb_triggered);

  trajectory->mutable_driving_state()->CopyFrom(driving_state);

  trajectory->set_lane_change_stage(lane_change_state.stage());
  if (lane_change_state.stage() != LCS_NONE) {
    trajectory->set_lane_change_left(lane_change_state.lc_left());
  }

  // TODO(guoqiang): Validate trajectories in planner_module.
  trajectory->mutable_traj_validation_result()->CopyFrom(validate_result);
}

void ConvertTrajectoryToGlobalCoordinates(
    const CoordinateConverter &coordinate_converter,
    const TrajectoryProto &trajectory,
    std::vector<PlannerState::PosePoint> *previous_trajectory_global,
    std::vector<PlannerState::PosePoint> *previous_past_trajectory_global) {
  // ---------------------------
  // Copy from planner_module.cc

  previous_trajectory_global->clear();
  for (const auto &trajectory_point : trajectory.trajectory_point()) {
    const auto &path_point = trajectory_point.path_point();
    const Vec2d smooth_point(path_point.x(), path_point.y());
    const Vec2d global_point =
        coordinate_converter.SmoothToGlobal(smooth_point);
    const double global_yaw =
        coordinate_converter.SmoothYawToGlobalNoNormalize(path_point.theta());
    previous_trajectory_global->push_back(
        {.pos = Vec2d(global_point.x(), global_point.y()),
         .theta = global_yaw});
  }

  // NOTE: trajectory.past_points() might be empty.
  // TODO(weijun): check trajectory.past_points()
  previous_past_trajectory_global->clear();
  for (const auto &trajectory_point : trajectory.past_points()) {
    const auto &path_point = trajectory_point.path_point();
    const Vec2d smooth_point(path_point.x(), path_point.y());
    const Vec2d global_point =
        coordinate_converter.SmoothToGlobal(smooth_point);
    const double global_yaw =
        coordinate_converter.SmoothYawToGlobalNoNormalize(path_point.theta());
    previous_past_trajectory_global->push_back(
        {.pos = Vec2d(global_point.x(), global_point.y()),
         .theta = global_yaw});
  }
}

void ReportCandidateTrafficLightInfo(
    const TrafficLightInfoMap &traffic_light_map, PlannerDebugProto *debug) {
  QCHECK_NOTNULL(debug);
  for (auto iter = traffic_light_map.begin(); iter != traffic_light_map.end();
       ++iter) {
    iter->second.ToProto(debug->add_candidate_traffic_light_info());
  }
}

void ReportSelectedTrafficLightInfo(
    const TrafficLightInfoMap &traffic_light_map,
    const mapping::LanePath &lane_path, TrafficLightInfoProto *proto) {
  for (const auto id : lane_path.lane_ids()) {
    const auto iter = traffic_light_map.find(id);
    if (iter != traffic_light_map.end()) {
      iter->second.ToProto(proto);
      return;
    }
  }
}

// Prev traj includes past points. Prev traj will never be empty once the
// first successful planner iteration completes.
std::vector<ApolloTrajectoryPointProto> CreatePreviousTrajectory(
    absl::Time plan_time, const TrajectoryProto &previous_trajectory,
    const MotionConstraintParamsProto &motion_constraint_params) {
  SCOPED_QTRACE("CreatePrevTrajectory");

  if (previous_trajectory.trajectory_point().empty()) return {};

  const double now_in_sec = ToUnixDoubleSeconds(plan_time);
  const double time_advancement = std::max(
      0.0, now_in_sec - previous_trajectory.trajectory_start_timestamp());
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory_points(
      previous_trajectory.trajectory_point().begin(),
      previous_trajectory.trajectory_point().end());
  return ShiftTrajectoryByTime(time_advancement, previous_trajectory_points,
                               motion_constraint_params.max_decel_jerk(),
                               motion_constraint_params.max_accel_jerk());
}  // namespace

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>> FallBackToPreviousPlan(
    const std::vector<ApolloTrajectoryPointProto> &time_aligned_prev_traj,
    bool reset) {
  const bool fallback_plan_valid =
      time_aligned_prev_traj.size() == kTrajectorySteps;
  if (!fallback_plan_valid) {
    return absl::UnavailableError("Previous trajectory not available.");
  }
  if (reset) {
    return absl::FailedPreconditionError(
        "Previous trajectory not valid when reset.");
  }

  return time_aligned_prev_traj;
}

bool NeedForceResetEstPlanner(const TrajectoryProto &prev_trajectory,
                              const AutonomyStateProto &now_autonomy_state,
                              const AutonomyStateProto &prev_autonomy_state,
                              bool previously_triggered_aeb,
                              bool is_emergency_stop, bool rerouted,
                              bool full_stopped,
                              std::vector<std::string> *reset_reasons) {
  bool force_reset = false;
  if (rerouted) {
    force_reset = true;
    reset_reasons->push_back("rerouted");
  }

  if (prev_trajectory.trajectory_point().empty()) {
    force_reset = true;
    reset_reasons->push_back("first-iteration");
  }
  if (!IS_AUTO_DRIVE(now_autonomy_state.autonomy_state())) {
    force_reset = true;
    reset_reasons->push_back("non-autonomy");
  }
  if (IS_ENGAGE(prev_autonomy_state.autonomy_state(),
                now_autonomy_state.autonomy_state())) {
    force_reset = true;
    reset_reasons->push_back("engage");
  }

  if (full_stopped) {
    force_reset = true;
    reset_reasons->push_back("fully stopped");
  }

  if (previously_triggered_aeb && !is_emergency_stop) {
    VLOG(2) << "Planner resetting when recovering from emergency stop";
    QEVENT("renjie", "planner_reset_exit_AEB", [](QEvent *qevent) {});
    force_reset = true;
    reset_reasons->push_back("recovering from emergency stop");
  }

  return force_reset;
}

void ReportHmiContent(const RouteContentProto &route_content,
                      const SemanticMapManager &semantic_map_manager,
                      const mapping::LanePath &lane_path,
                      HmiContentProto *hmi) {
  *hmi->mutable_route_content() = route_content;

  // Writing lane path direction info to hmi turning message
  int turning = 0;
  if (!lane_path.IsEmpty()) {
    const auto &current_lane_id = lane_path.front().lane_id();
    const mapping::LaneProto &current_lane_proto =
        semantic_map_manager.FindLaneByIdOrDie(current_lane_id);

    if (current_lane_proto.has_direction()) {
      const auto &direction = current_lane_proto.direction();
      switch (direction) {
        case mapping::LaneProto::LEFT_TURN:
          turning = 1;
          break;
        case mapping::LaneProto::RIGHT_TURN:
          turning = -1;
          break;
        case mapping::LaneProto::UTURN:
          turning = 1;
          break;
        case mapping::LaneProto::STRAIGHT:
          break;
      }
    }
  }

  hmi->set_turning(turning);
}

void ReportTeleopStatus(const TeleopState &teleop_state,
                        const SemanticMapManager &semantic_map_manager,
                        const CoordinateConverter &coordinate_converter,
                        PlannerTeleopStatusProto *teleop_status) {
  if (teleop_state.IsOverrideLeftBlinker()) {
    teleop_status->mutable_left_blinker_override_status()->set_on(
        teleop_state.IsOverrideLeftBlinkerOn());
  }
  if (teleop_state.IsOverrideRightBlinker()) {
    teleop_status->mutable_right_blinker_override_status()->set_on(
        teleop_state.IsOverrideRightBlinkerOn());
  }
  if (teleop_state.lane_change_direction.has_value()) {
    teleop_status->mutable_lane_change_status()->set_direction(
        *teleop_state.lane_change_direction);
  }
  if (teleop_state.IsOverrideEmergencyBlinker()) {
    teleop_status->mutable_emergency_blinker_override_status()->set_on(
        teleop_state.IsOverrideEmergencyBlinkerOn());
  }

  teleop_status->mutable_enable_feature_status()
      ->set_traffic_light_stopping_is_enabled(
          teleop_state.IsEnableTrafficLightStopping());
  teleop_status->mutable_enable_feature_status()->set_lc_objects_is_enabled(
      teleop_state.IsEnableLcObjects());

  teleop_status->mutable_enable_feature_status()->set_pull_over_is_enabled(
      teleop_state.IsEnablePullOver());

  teleop_status->set_brake_to_stop(teleop_state.BrakeToStop());
}

absl::StatusOr<mapping::LanePath> FindRouteLanePathFromStartPoint(
    const PlannerSemanticMapManager &psmm,
    const mapping::LanePath &raw_lane_path, double projection_range,
    const ApolloTrajectoryPointProto &start_point, double *travel_arc_len) {
  const Vec2d start_pos(start_point.path_point().x(),
                        start_point.path_point().y());

  ASSIGN_OR_RETURN(
      const auto lane_pt,
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm.GetLevel(), *psmm.semantic_map_manager(), start_pos,
          raw_lane_path.BeforeArclength(projection_range),
          start_point.path_point().theta()));
  if (!raw_lane_path.ContainsLanePoint(lane_pt)) {
    return absl::NotFoundError("Unable to find lane point on lane path.");
  }
  const double arc_len =
      raw_lane_path.FirstOccurrenceOfLanePointToArclength(lane_pt);
  if (travel_arc_len != nullptr) *travel_arc_len = arc_len;
  return raw_lane_path.AfterArclength(arc_len);
}

PlanStartPointInfo ComputeEstPlanStartPoint(
    absl::Time predicted_plan_time, const TrajectoryProto &prev_trajectory,
    const PoseProto &pose, const AutonomyStateProto &now_autonomy_state,
    const AutonomyStateProto &prev_autonomy_state,
    bool previously_triggered_aeb, bool rerouted, bool aeb,
    const Chassis &chassis,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &vehicle_geom_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  SCOPED_QTRACE("ComputeEstPlanStartPoint");

  std::vector<std::string> reset_reasons;
  std::optional<ApolloTrajectoryPointProto> prev_planned_traj_point =
      std::nullopt;
  bool reset = false;
  double path_s_increment_from_previous_frame = 0.0;
  absl::Time plan_start_time = predicted_plan_time;
  const std::optional<int> start_index_on_prev_traj =
      InterpolatePointFromPrevTrajectory(predicted_plan_time, prev_trajectory);

  if (!start_index_on_prev_traj.has_value()) {
    reset = true;
    reset_reasons.push_back("prev_plan_point not found");
  } else {
    prev_planned_traj_point = std::make_optional<ApolloTrajectoryPointProto>(
        prev_trajectory.trajectory_point(*start_index_on_prev_traj));
    plan_start_time =
        FromUnixDoubleSeconds(prev_planned_traj_point->relative_time() +
                              prev_trajectory.trajectory_start_timestamp());
    prev_planned_traj_point->set_relative_time(0.0);
    path_s_increment_from_previous_frame =
        prev_planned_traj_point->path_point().s();
    prev_planned_traj_point->mutable_path_point()->set_s(0.0);

    // Check if we need reset.
    // Trajectory point from previous trajectory at Clock::Now().
    ApolloTrajectoryPointProto prev_planned_now_point;
    if (!InterpolatePointFromPrevTrajectoryIncludingPast(
            FromUnixDoubleSeconds(pose.timestamp()), prev_trajectory,
            &prev_planned_now_point)) {
      // Reset if prev_planned_now_point not found.
      reset = true;
      reset_reasons.push_back("prev_now_point not found");
    } else {
      const Vec2d current_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
      // Reset if control error too large.
      reset = MaybeResetEstPlanner(prev_planned_now_point, current_pos,
                                   &reset_reasons);
    }
  }

  constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
  const bool full_stop =
      prev_planned_traj_point.has_value() &&
      prev_planned_traj_point->v() == 0.0 &&
      std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;

  if (!reset) {
    reset = NeedForceResetEstPlanner(
        prev_trajectory, now_autonomy_state, prev_autonomy_state,
        previously_triggered_aeb, aeb, rerouted, full_stop, &reset_reasons);
  }

  if (!reset) {
    QCHECK(prev_planned_traj_point.has_value());
  }
  return PlanStartPointInfo{
      .reset = reset,
      .start_index_on_prev_traj = start_index_on_prev_traj,
      .start_point = reset ? ComputePlanStartPointAfterReset(
                                 prev_planned_traj_point, pose, chassis,
                                 motion_constraint_params, vehicle_geom_params,
                                 vehicle_drive_params)
                           : *prev_planned_traj_point,
      .path_s_increment_from_previous_frame =
          reset ? 0.0 : path_s_increment_from_previous_frame,
      .plan_time =
          reset ? FromUnixDoubleSeconds(pose.timestamp()) : plan_start_time,
      .full_stop = full_stop,
      .reasons = std::move(reset_reasons)};
}

absl::Duration GetStPathPlanLookAheadTime(
    const PlanStartPointInfo &plan_start_point_info, const PoseProto &pose,
    const TrajectoryProto &previous_trajectory) {
  constexpr double kLookAheadTimeMaxTime = 5.0;  // s
  absl::Duration look_ahead_time =
      absl::Milliseconds(FLAGS_planner_deferred_path_planning_time_ms);
  // Since we have spacetime planning but lateral and longitudinal control,
  // first reference point for laterl control may have large time diff with plan
  // start point. Notice that after getting spacetime trajectory, speed planning
  // using spacetime result as path will be running. If we start plan path from
  // point close to control ref point, there will be a performance improvement
  // on lateral control. So We check the closest point of pose on previous
  // trajectory and check the diff between it and plan start point, if too
  // large, set the path plan start point to this point.
  // https://docs.google.com/presentation/d/1oXnBZ95lRy_X1U9U_dPWJ_a8MPZSGTSRJuPeqzrrhiM/edit#slide=id.p1
  if (FLAGS_enable_path_start_point_look_ahead &&
      plan_start_point_info.start_index_on_prev_traj.has_value() &&
      !previous_trajectory.trajectory_point().empty()) {
    const auto &previous_trajectory_points =
        previous_trajectory.trajectory_point();
    const Vec2d plan_start_pos =
        Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y());
    const auto closest_iter = absl::c_min_element(
        previous_trajectory_points,
        [&plan_start_pos](const ApolloTrajectoryPointProto &p1,
                          const ApolloTrajectoryPointProto &p2) {
          const Vec2d pos1(p1.path_point().x(), p1.path_point().y());
          const Vec2d pos2(p2.path_point().x(), p2.path_point().y());
          return (pos1 - plan_start_pos).squaredNorm() <
                 (pos2 - plan_start_pos).squaredNorm();
        });
    const int closest_index_on_prev_traj =
        std::distance(previous_trajectory_points.begin(), closest_iter);
    const auto diff_time =
        static_cast<double>((closest_index_on_prev_traj -
                             *plan_start_point_info.start_index_on_prev_traj) *
                            kTrajectoryTimeStep);
    if (diff_time > FLAGS_planner_path_start_point_time_diff_limit) {
      look_ahead_time +=
          absl::Seconds(std::min(kLookAheadTimeMaxTime, diff_time));
    }
  }
  return look_ahead_time;
}

StPathPlanStartPointInfo GetStPathPlanStartPointInfo(
    const absl::Duration look_ahead_time,
    const PlanStartPointInfo &plan_start_point_info,
    const TrajectoryProto &previous_trajectory) {
  absl::Time path_planning_time =
      plan_start_point_info.plan_time + look_ahead_time;
  const auto path_start_index_on_prev_traj = InterpolatePointFromPrevTrajectory(
      path_planning_time, previous_trajectory);
  if (plan_start_point_info.reset ||
      !path_start_index_on_prev_traj.has_value()) {
    VLOG(3) << "Do not change path plan start time because planner reset or "
               "path planning time not found in previous trajectory.";
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point = 0,
            .start_point = plan_start_point_info.start_point,
            .plan_time = plan_start_point_info.plan_time};
  } else {
    VLOG(3) << "Use FLAGS_planner_deferred_path_planning_time_ms("
            << FLAGS_planner_deferred_path_planning_time_ms
            << "ms) deferred planning "
            << "path_planning_time:" << path_planning_time;
    ApolloTrajectoryPointProto path_plan_start_point =
        previous_trajectory.trajectory_point(*path_start_index_on_prev_traj);
    path_plan_start_point.set_relative_time(0.0);
    path_plan_start_point.mutable_path_point()->set_s(0.0);
    // If planner not reset, plan_start_point_info.start_index_on_prev_traj
    // should have value.
    QCHECK(plan_start_point_info.start_index_on_prev_traj.has_value());
    return {.reset = plan_start_point_info.reset,
            .relative_index_from_plan_start_point =
                *path_start_index_on_prev_traj -
                *plan_start_point_info.start_index_on_prev_traj,
            .start_point = path_plan_start_point,
            .plan_time = path_planning_time};
  }
}

absl::StatusOr<mapping::LanePath> FindPreferredLanePathFromTeleop(
    const PlannerSemanticMapManager &psmm,
    const RouteSections &route_sections_from_start,
    const absl::flat_hash_set<mapping::ElementId> &avoid_lanes, double ego_v,
    const PointOnRouteSections &ego_proj,
    const LaneChangeRequestProto &lc_request) {
  const auto &lane_ids =
      psmm.FindSectionInfoOrDie(route_sections_from_start.front().id).lane_ids;
  const auto current_it =
      std::find(lane_ids.begin(), lane_ids.end(), ego_proj.lane_id);
  if (current_it == lane_ids.end()) {
    return absl::NotFoundError(
        "Current lane not found in the current route section!");
  }

  if (lc_request.direction() == LaneChangeRequestProto::STRAIGHT) {
    const RouteSectionsInfo sections_info(psmm, &route_sections_from_start,
                                          avoid_lanes);
    return FindLanePathFromLaneAlongRouteSections(
        psmm, sections_info, *current_it, sections_info.length());
  }

  constexpr double kMinKeepLength = 100.0;       // m.
  constexpr double kKeepLengthExtension = 10.0;  // m.
  const double keep_len =
      std::max(kMinKeepLength, ego_v * kTrajectoryTimeHorizon);
  const auto short_route_sections = *ClampRouteSectionsBeforeArcLength(
      psmm, route_sections_from_start, keep_len + kKeepLengthExtension);
  const RouteSectionsInfo short_sections_info(psmm, &short_route_sections,
                                              avoid_lanes);

  if (lc_request.direction() == LaneChangeRequestProto::LEFT) {
    if (current_it == lane_ids.begin()) {
      return absl::NotFoundError("Already on the leftmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, *std::prev(current_it), keep_len);
  } else {
    if (std::next(current_it) == lane_ids.end()) {
      return absl::NotFoundError("Already on the rightmost lane!");
    }
    return FindLanePathFromLaneAlongRouteSections(
        psmm, short_sections_info, *std::next(current_it), keep_len);
  }
}

absl::Status TranslateToStationaryTrajectory(const PoseProto &pose,
                                             TrajectoryProto *trajectory) {
  PathPoint start_path_point;
  start_path_point.set_x(pose.pos_smooth().x());
  start_path_point.set_y(pose.pos_smooth().y());
  start_path_point.set_z(pose.pos_smooth().z());
  start_path_point.set_theta(pose.yaw());

  // Rewrite forward trajectory.
  trajectory->mutable_trajectory_point()->Clear();
  trajectory->mutable_trajectory_point()->Reserve(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = start_path_point;
    traj_point.set_relative_time(i * kTrajectoryTimeStep);
    *trajectory->add_trajectory_point() = std::move(traj_point);
  }

  // Rewrite past trajectory .
  trajectory->mutable_past_points()->Clear();
  trajectory->mutable_past_points()->Reserve(kMaxPastPointNum);
  for (int i = -kMaxPastPointNum; i < 0; ++i) {
    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = start_path_point;
    traj_point.set_relative_time(i * kTrajectoryTimeStep);
    *trajectory->add_past_points() = std::move(traj_point);
  }
  return absl::OkStatus();
}
}  // namespace planner
}  // namespace qcraft
