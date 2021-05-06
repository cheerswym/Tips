#include "onboard/planner/plan/cruise_task.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/auto_tuning_common_flags.h"
#include "onboard/async/async_macro.h"
#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/planner/decision/door_open_decider.h"
#include "onboard/planner/decision/traffic_light_info_collector.h"
#include "onboard/planner/decision/turn_signal_decider.h"
#include "onboard/planner/driving_state.h"
#include "onboard/planner/plan/aeb_planner.h"
#include "onboard/planner/plan/multi_tasks_est_planner.h"
#include "onboard/planner/plan/previous_trajectory_planner.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scene/scene_reasoning_util.h"
#include "onboard/planner/scene/scene_understanding.h"
#include "onboard/planner/scheduler/local_map_builder.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/planner_status_macros.h"

DEFINE_int32(
    prev_traj_planner_max_frame, 3,
    "The max frame number that the previous-trajectory planner is allowed "
    "to be consecutively used. If set to zero, the previous-trajectory planner "
    "is disabled.");

namespace qcraft::planner {

namespace {

void ParseEstPlannerDebugToProto(TurnSignalResult turn_signal_res,
                                 const PlannerStatus &planner_status,
                                 SchedulerOutput scheduler_output,
                                 EstPlannerDebug est_debug,
                                 EstPlannerDebugProto *est_planner_debug) {
  est_planner_debug->set_turn_signal_reason_enum(turn_signal_res.reason);

  planner_status.ToProto(est_planner_debug->mutable_planner_status());

  *est_planner_debug->mutable_filtered() =
      std::move(est_debug.filtered_prediction_trajectories);

  *est_planner_debug->mutable_st_planner_trajectories() =
      std::move(est_debug.st_planner_trajectories);

  ToSchedulerOutputProto(std::move(scheduler_output),
                         est_planner_debug->mutable_scheduler());

  *est_planner_debug->mutable_constraint() =
      std::move(est_debug.decision_constraints);

  *est_planner_debug->mutable_initializer() =
      std::move(est_debug.initializer_debug_proto);

  *est_planner_debug->mutable_trajectory_optimizer() =
      std::move(est_debug.optimizer_debug_proto);

  *est_planner_debug->mutable_speed_finder() =
      std::move(est_debug.speed_finder_debug);

  *est_planner_debug->mutable_traj_validation() =
      std::move(est_debug.traj_validation_result);
}

void ParseEstPlannerOutputToPlannerState(EstPlannerOutput est_planner_output,
                                         PlannerState *planner_state) {
  planner_state->decider_state = std::move(est_planner_output.decider_state);
  planner_state->initializer_state =
      std::move(est_planner_output.initializer_state);
  planner_state->st_planner_trajectories =
      std::move(est_planner_output.st_planner_trajectories);
}

// No valid output but still contains some debug information.
bool IsNonEmptyPlannerResult(const PlannerStatus &status) {
  switch (status.status_code()) {
    case PlannerStatusProto::OK:
    case PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE:
    case PlannerStatusProto::INITIALIZER_FAILED:
    case PlannerStatusProto::OPTIMIZER_FAILED:
    case PlannerStatusProto::PATH_EXTENSION_FAILED:
    case PlannerStatusProto::SPEED_OPTIMIZER_FAILED:
    case PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED:
    case PlannerStatusProto::SELECTOR_FAILED:
      return true;
    case PlannerStatusProto::ROUTE_MSG_UNAVAILABLE:
    case PlannerStatusProto::INPUT_INCORRECT:
    case PlannerStatusProto::ROUTING_FAILED:
    case PlannerStatusProto::OBJECT_MANAGER_FAILED:
    case PlannerStatusProto::REFERENCE_PATH_UNAVAILABLE:
    case PlannerStatusProto::SCHEDULER_UNAVAILABLE:
    case PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE:
    case PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED:
    case PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED:
    case PlannerStatusProto::PLANNER_ABNORMAL_EXIT:
    case PlannerStatusProto::LOCAL_MAP_BUILDER_FAILED:
    case PlannerStatusProto::SCENE_UNDERSTANDING_FAILED:
    case PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED:
    case PlannerStatusProto::PLANNER_STATE_INCOMPLETE:
    case PlannerStatusProto::EXPERT_TRAJ_UNAVAILABLE:
    case PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED:
    case PlannerStatusProto::EXPERT_SPEED_IN_CONTRADICTORY_WITH_SPEED_FINDER:
    case PlannerStatusProto::PATH_PLANNER_FAILED:
      return false;
  }
}

}  // namespace

PlannerStatus RunCruiseTask(const CruiseTaskInput &input,
                            CruiseTaskOutput *result,
                            PlannerState *planner_state,
                            ThreadPool *thread_pool, LiteModule *lite_module) {
  SCOPED_QTRACE("OnRoadPlanningMainLoop");

  // Initialize route sections.
  if (planner_state->prev_route_sections.empty() || input.rerouted) {
    const Vec2d pos = Vec2dFromApolloTrajectoryPointProto(
        input.plan_start_point_info->start_point);
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        planner_state->prev_route_sections,
        BackwardExtendRouteSectionsFromPos(
            *input.planner_input->planner_semantic_map_manager,
            ConvertRouteSectionSequenceToRouteSections(
                input.route_output->route_from_current->section_sequence()),
            pos, kDrivePassageKeepBehindLength),
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);
    auto target_lane_path_or = FindClosestTargetLanePathOnReset(
        *input.planner_input->planner_semantic_map_manager,
        planner_state->prev_route_sections, pos);

    if (target_lane_path_or.ok()) {
      planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
    } else {
      // BANDAID(SCENARIOS-689): Force to recalculate prev target lane path by
      // clearing prev route section.
      planner_state->prev_route_sections.Clear();
      return PlannerStatus(
          PlannerStatusProto::RESET_PREV_TARGET_LANE_PATH_FAILED,
          target_lane_path_or.status().message());
    }
  }

  if (planner_state->prev_target_lane_path.IsEmpty()) {
    // Should have been filled at the end of the last iteration.
    return PlannerStatus(PlannerStatusProto::PLANNER_STATE_INCOMPLETE,
                         "Prev target lane path empty.");
  }

  const absl::Cleanup fill_prev_target_lane_path = [&input, &planner_state]() {
    // Check and fill prev_target_lane_path before the end of each iteration.
    if (planner_state->prev_target_lane_path.IsEmpty()) {
      auto target_lane_path_or = FindClosestTargetLanePathOnReset(
          *input.planner_input->planner_semantic_map_manager,
          planner_state->prev_route_sections,
          Vec2dFromApolloTrajectoryPointProto(
              input.plan_start_point_info->start_point));
      if (target_lane_path_or.ok()) {
        planner_state->prev_target_lane_path = std::move(*target_lane_path_or);
      }
    }
  };

  PlannerDebugProto debug_proto;

  // ----------------------------------------------------------
  // -------------------- Route Section -----------------------
  // ----------------------------------------------------------
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto route_sections_proj,
      ProjectPointToRouteSections(
          *input.planner_input->planner_semantic_map_manager,
          planner_state->prev_route_sections,
          Vec2dFromApolloTrajectoryPointProto(
              input.plan_start_point_info->start_point),
          kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          kDrivePassageKeepBehindLength),
      PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);
  auto [route_sections_from_start, route_sections_with_behind, ego_pos_proj] =
      std::move(route_sections_proj);
  planner_state->prev_route_sections = std::move(route_sections_with_behind);

  // ----------------------------------------------------------
  // -------------------- Smooth Reference Line -----------------------
  // ----------------------------------------------------------
  if (FLAGS_planner_save_smooth_result_in_planner_state) {
    const double half_av_width =
        input.planner_input->vehicle_params.vehicle_geometry_params().width() *
        0.5;
    const auto smooth_result_map = BuildSmoothedResultMapFromRouteSections(
        *input.planner_input->planner_semantic_map_manager,
        route_sections_from_start, half_av_width,
        std::move(planner_state->smooth_result_map));
    if (smooth_result_map.ok()) {
      planner_state->smooth_result_map = std::move(*smooth_result_map);
    }
  }

  // ----------------------------------------------------------
  // ----------- Traffic light info collect  ------------------
  // ----------------------------------------------------------

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto tl_info_collector_output,
      CollectTrafficLightInfo(
          TrafficLightInfoCollectorInput{
              .psmm = input.planner_input->planner_semantic_map_manager.get(),
              .traffic_light_states =
                  input.planner_input->traffic_light_states.get(),
              .route_sections = &route_sections_from_start,
              .plan_time = input.plan_time},
          std::move(planner_state->yellow_light_observations)),
      PlannerStatusProto::TRAFFIC_LIGHT_INFO_COLLECTOR_FAILED);
  planner_state->yellow_light_observations =
      std::move(tl_info_collector_output.yellow_light_observations);

  // ----------------------------------------------------------
  // -------------------Scene Understanding -------------------
  // ----------------------------------------------------------
  absl::flat_hash_set<std::string> stalled_objects;

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto lane_paths,
      BuildLocalMap(*input.planner_input->planner_semantic_map_manager,
                    route_sections_from_start,
                    input.route_output->route->avoid_lanes()),
      PlannerStatusProto::LOCAL_MAP_BUILDER_FAILED);

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto scene_understanding_result,
      RunSceneUnderstanding(
          SceneUnderstandInput{
              .psmm = input.planner_input->planner_semantic_map_manager.get(),
              .prediction = input.planner_input->prediction.get(),
              .tl_info_map = &tl_info_collector_output.tl_info_map,
              .lane_paths = &lane_paths,
              .sensor_fovs = input.planner_input->sensor_fovs.get(),
              .route_sections = &route_sections_from_start},
          thread_pool),
      PlannerStatusProto::SCENE_UNDERSTANDING_FAILED);
  *debug_proto.mutable_scene_understanding_debug() = scene_understanding_result;

  // Record stalled objects.
  ParseObjectAnnotationToDebugProto(
      scene_understanding_result.objects_annotation(), *input.object_manager,
      &debug_proto);

  // Record traffic waiting objects.
  ParseTrafficWaitingQueueToDebugProto(
      scene_understanding_result.traffic_waiting_queue(), *input.object_manager,
      &debug_proto);

  for (const auto &stalled_object : debug_proto.stalled_objects()) {
    stalled_objects.insert(stalled_object.id());
  }

  // ----------------------------------------------------------
  // --------------------- AEB --------------------------------
  // ----------------------------------------------------------
  // TODO(perf opt master): parallel running.
  Future<absl::StatusOr<AebPlannerOutput>> future_aeb_result =
      ScheduleFuture(thread_pool, [&]() {
        Polygon2d aeb_risk_area;
        auto aeb_plan_result_or = RunAebPlanner(
            *input.planner_input, input.plan_start_point_info->start_point,
            input.plan_start_point_info->path_s_increment_from_previous_frame,
            input.plan_start_point_info->reset,
            *input.time_aligned_prev_traj_points, *input.planner_params,
            &aeb_risk_area);
        if (aeb_risk_area.num_points() > 0) {
          for (const auto &point : aeb_risk_area.points()) {
            Vec2dProto *vec = debug_proto.add_aeb_detection_polygon();
            vec->set_x(point.x());
            vec->set_y(point.y());
          }
        }
        return aeb_plan_result_or;
      });

  vis::vantage::ChartsDataProto chart_data;

  // ------------------------------------------------------------------
  // ------------------ Previous trajectory planner -------------------
  // ------------------------------------------------------------------
  Future<absl::StatusOr<PreviousTrajectoryPlannerOutput>>
      future_prev_traj_result = ScheduleFuture(thread_pool, [&]() {
        return RunPreviousTrajectoryPlanner(
            *input.planner_input->planner_semantic_map_manager
                 ->semantic_map_manager(),
            *input.planner_input->pose,
            input.planner_input->vehicle_params.vehicle_geometry_params(),
            input.planner_input->vehicle_params.vehicle_drive_params(),
            input.planner_params->planner_params().motion_constraint_params(),
            *input.time_aligned_prev_traj_points);
      });

  // ------------------------------------------------------------------
  // ------------------ Preferred lanes from teleop -------------------
  // ------------------------------------------------------------------
  if (!planner_state->preferred_lane_path.IsEmpty()) {
    const auto ff_or = BuildBruteForceFrenetFrame(
        SampleLanePathPoints(*input.planner_input->planner_semantic_map_manager
                                  ->semantic_map_manager(),
                             planner_state->preferred_lane_path.BeforeArclength(
                                 kMaxTravelDistanceBetweenFrames)));
    if (!ff_or.ok()) {
      // Clear preferred lane path if its end is reached.
      planner_state->preferred_lane_path = mapping::LanePath();
      QLOG(INFO) << "Teleop lane change state released!";
    } else {
      const auto ego_sl = ff_or->XYToSL(Vec2dFromApolloTrajectoryPointProto(
          input.plan_start_point_info->start_point));
      planner_state->preferred_lane_path =
          planner_state->preferred_lane_path.AfterArclength(ego_sl.s);
    }
  }
  if (!input.teleop_state->queued_lane_change_requests.empty()) {
    const auto &lc_request =
        input.teleop_state->queued_lane_change_requests.front();
    if (lc_request.direction() == LaneChangeRequestProto::CANCEL) {
      planner_state->preferred_lane_path = mapping::LanePath();
      QLOG(INFO) << "Cleared teleop lane change state!";
    } else {
      auto preferred_lane_path_or = FindPreferredLanePathFromTeleop(
          *input.planner_input->planner_semantic_map_manager,
          route_sections_from_start, input.route_output->route->avoid_lanes(),
          input.plan_start_point_info->start_point.v(), ego_pos_proj,
          lc_request);
      if (!preferred_lane_path_or.ok()) {
        QLOG(WARNING) << "Setting teleop lane change state failed: "
                      << preferred_lane_path_or.status().message();
      } else {
        // New teleop command incoming, overwrite the existing one.
        planner_state->preferred_lane_path = std::move(*preferred_lane_path_or);
      }
    }
  }

  // ----------------------------------------------------------
  // --------------------- Est planner ------------------------
  // ----------------------------------------------------------
  auto est_result = std::make_unique<PathBoundedEstPlannerOutput>();
  PlannerStatus est_status;

  const MultiTasksEstPlannerInput multi_task_est_planner_input{
      .planner_input = input.planner_input,
      .planner_params = input.planner_params,
      .route = input.route_output->route.get(),
      .route_sections_from_start = &route_sections_from_start,
      .start_point_info = input.plan_start_point_info,
      .st_traj_mgr = input.st_traj_mgr,
      .object_manager = input.object_manager,
      .stalled_objects = &stalled_objects,
      .scene_reasoning = &scene_understanding_result,
      .teleop_state = input.teleop_state,
      .planner_state = planner_state,
      .time_aligned_prev_traj = input.time_aligned_prev_traj_points,
      .prev_target_lane_path = &planner_state->prev_target_lane_path,
      .prev_route_sections = &planner_state->prev_route_sections,
      .tl_info_map = &tl_info_collector_output.tl_info_map,
      .station_anchor = &planner_state->station_anchor,
      .smooth_result_map = &planner_state->smooth_result_map,
      .prev_smooth_state = planner_state->prev_smooth_state,
      .sensor_fovs = input.planner_input->sensor_fovs.get()};
  est_status = RunMultiTasksEstPlanner(multi_task_est_planner_input,
                                       est_result.get(), thread_pool);

  if (est_status.ok()) {
    *debug_proto.mutable_selector_debug() =
        std::move(est_result->selector_debug);
    *(debug_proto.mutable_auto_tuning_data()->mutable_expert_costs()) =
        est_result->est_planner_output_list[0].expert_costs;
    for (const auto &est_planner_output : est_result->est_planner_output_list) {
      *(debug_proto.mutable_auto_tuning_data()->add_costs()) =
          std::move(est_planner_output.feature_costs);
    }
    for (const auto &est_planner_debug : est_result->est_planner_debug_list) {
      const auto &traj_opt_result_points =
          est_planner_debug.optimizer_debug_proto.ddp().final_traj();
      if (kDdpTrajectoryStepsDAT <= traj_opt_result_points.size()) {
        TrajectoryProto traj_proto;
        for (int i = 0; i < kDdpTrajectoryStepsDAT; ++i) {
          *(traj_proto.add_trajectory_point()) =
              ToApolloTrajectoryPointProto(traj_opt_result_points[i]);
        }
        *(debug_proto.mutable_auto_tuning_data()->add_trajectories()) =
            std::move(traj_proto);
      } else {
        QLOG(ERROR) << "Optimizer output trajectory should has at least "
                    << kDdpTrajectoryStepsDAT << " points but only has "
                    << traj_opt_result_points.size() << " points.";
      }
    }
  }

  // ------------------------------------------------------------
  // --------------------- Select result ------------------------
  // ------------------------------------------------------------
  PlannerStatus on_road_plan_status = OkPlannerStatus();
  std::vector<ApolloTrajectoryPointProto> planned_traj_points;

  // TODO(lidong): Find a method to wait all futures asynchronously.
  auto aeb_result = future_aeb_result.Get();
  auto prev_traj_result = future_prev_traj_result.Get();
  const auto &fallback_status = est_result->fallback_status;

  // Clear planner state, should be filled by the active planner.
  planner_state->prev_target_lane_path = mapping::LanePath();
  planner_state->prev_smooth_state = false;

  if (aeb_result.ok()) {
    QEVENT_EVERY_N_SECONDS("ping", "AEB_triggered", 5.0, [&](QEvent *qevent) {
      qevent->AddField("object_id", aeb_result->aeb_object_id)
          .AddField("object_type",
                    ObjectType_Name(aeb_result->aeb_object_type));
    });
    debug_proto.mutable_emergency_stop()->set_is_enabled(true);
    debug_proto.set_active_planner(AEB_PLANNER);
    planned_traj_points = std::move(aeb_result->trajectory_points);
    planner_state->previous_trajectory_plan_counter = 0;
  } else if (est_status.ok()) {
    planned_traj_points = est_result->est_planner_output_list[0].traj_points;
    const auto &selected_scheduler_output =
        est_result->scheduler_output_list[0];

    if (!selected_scheduler_output.is_fallback) {
      debug_proto.set_active_planner(EST_PLANNER);
    } else {
      QEVENT("renjie", "use_fallback_planner", [&](QEvent *qevent) {});
      debug_proto.set_active_planner(FALLBACK_PLANNER);
    }

    planner_state->previous_trajectory_plan_counter = 0;

    const auto &drive_passage = selected_scheduler_output.drive_passage;
    planner_state->prev_target_lane_path =
        drive_passage.extend_lane_path().BeforeLastOccurrenceOfLanePoint(
            drive_passage.lane_path().back());
    planner_state->station_anchor =
        drive_passage.FindNearestStationAtS(0.0).GetLanePoint();

    planner_state->prev_length_along_route =
        selected_scheduler_output.length_along_route;
    planner_state->prev_smooth_state = selected_scheduler_output.should_smooth;
    // NOTE: The following fields are faked by multi-task scheduler and
    // should be removed in the future.
    planner_state->lane_change_state =
        selected_scheduler_output.lane_change_state;
    planner_state->prev_lane_path_before_lc =
        selected_scheduler_output.lane_path_before_lc;
  } else if (prev_traj_result.ok()) {
    QEVENT("renjie", "use_prev_traj_planner", [&](QEvent *qevent) {});
    QLOG(WARNING) << "Est planner fails: " << est_status.message();
    QLOG(WARNING) << "Fallback planner fails: " << fallback_status.message();
    QLOG(WARNING) << "Use previous-trajectory planner!";
    debug_proto.set_active_planner(PREV_TRAJ_PLANNER);
    planned_traj_points = std::move(prev_traj_result->trajectory_points);
    planner_state->previous_trajectory_plan_counter++;
  } else {
    QLOG(WARNING) << "Est planner fails: " << est_status.message();
    QLOG(WARNING) << "Fallback planner fails: " << fallback_status.message();
    QLOG(WARNING) << "Previous-trajectory planner fails: "
                  << prev_traj_result.status().ToString();
    debug_proto.set_active_planner(PREV_TRAJ_PLANNER);
    planner_state->previous_trajectory_plan_counter = 0;

    // Kickout if there is no valid plan.
    const std::string reason = absl::StrCat(
        "No valid plan.\n  Est planner error: ", est_status.message(),
        "  Fallback planner error: ", fallback_status.message(),
        "  Previous-trajectory error: ", prev_traj_result.status().ToString());
    QISSUEX_WITH_ARGS(QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
                      QIssueSubType::QIST_PLANNER_PLAN_INVALID,
                      "No valid plan. ", reason);
    on_road_plan_status = PlannerStatus(fallback_status.status_code(), reason);
  }

  // TODO(renjie): Delete the following kickout logic.
  // Kick out if planner falls back for too many consecutive frames.
  if (planner_state->previous_trajectory_plan_counter >
          FLAGS_prev_traj_planner_max_frame &&
      !OnTestBenchForRsim()) {
    const std::string reason =
        absl::StrCat("Previous-trajectory plan frames: ",
                     planner_state->previous_trajectory_plan_counter,
                     " Latest EstPlanner error: ", est_status.message());
    QISSUEX_WITH_ARGS(
        QIssueSeverity::QIS_ERROR, QIssueType::QIT_BUSINESS,
        QIssueSubType::QIST_PLANNER_TRAJECTORY_INVALID,
        "Use previous-trajectory plan for too many consecutive frames.",
        reason);
    on_road_plan_status = PlannerStatus(fallback_status.status_code(), reason);
  }

  //-----------------------------------------------------------------
  //--------------------- Fill trajectory proto ---------------------
  //-----------------------------------------------------------------
  TurnSignalResult turn_signal_result;
  DrivingStateProto driving_state;
  // TODO(weijun): Delete them.
  mapping::LanePath target_lane_path_from_current;
  LaneChangeStateProto lane_change_state;
  TrajectoryValidationResultProto traj_validation_result;
  if (est_status.ok()) {
    const auto &scheduler_output = est_result->scheduler_output_list[0];
    turn_signal_result =
        DecideTurnSignal(*input.planner_input->planner_semantic_map_manager,
                         input.route_output->signal, route_sections_from_start,
                         scheduler_output.drive_passage.lane_path(),
                         scheduler_output.lane_change_state,
                         *input.teleop_state, scheduler_output.drive_passage,
                         scheduler_output.av_frenet_box_on_drive_passage);

    driving_state = GetOnRoadDrivingState(
        input.planner_input->vehicle_params.vehicle_geometry_params(),
        input.plan_start_point_info->full_stop,
        scheduler_output.drive_passage.lane_path());

    target_lane_path_from_current = scheduler_output.drive_passage.lane_path(),

    lane_change_state = scheduler_output.lane_change_state;

    traj_validation_result =
        std::move(est_result->est_planner_debug_list[0].traj_validation_result);
  }

  //-----------------------------------------------------------------
  //--------------------- Set door decision -------------------------
  //-----------------------------------------------------------------
  const auto door_decision = ComputeDoorDecision(
      input.plan_time, planner_state->last_door_override_time,
      input.teleop_state->override_door, input.teleop_state->override_door_open,
      driving_state.type() == DrivingStateProto::STOPPED_AT_END_OF_ROUTE);

  TrajectoryProto trajectory_info;
  const auto past_points = CreatePastPointsList(
      input.plan_time, input.plan_start_point_info->start_point,
      planner_state->previous_trajectory, input.plan_start_point_info->reset);

  FillTrajectoryProto(input.plan_time, *input.route_output, planned_traj_points,
                      past_points, target_lane_path_from_current,
                      lane_change_state, turn_signal_result.signal,
                      door_decision, aeb_result.ok(), driving_state,
                      traj_validation_result, &trajectory_info);

  // -----------------------------------------------------------------
  // --------------------- Fill planner debug ------------------------
  // -----------------------------------------------------------------
  *debug_proto.mutable_fallback_planner_debug() =
      std::move(est_result->fallback_debug);
  if (IsNonEmptyPlannerResult(est_status)) {
    for (int plan_idx = 0; plan_idx < est_result->est_planner_debug_list.size();
         ++plan_idx) {
      const auto &scheduler_output =
          est_result->scheduler_output_list[plan_idx];
      auto turn_signal_res = DecideTurnSignal(
          *input.planner_input->planner_semantic_map_manager,
          input.route_output->signal, route_sections_from_start,
          scheduler_output.drive_passage.lane_path(),
          scheduler_output.lane_change_state, *input.teleop_state,
          scheduler_output.drive_passage,
          scheduler_output.av_frenet_box_on_drive_passage);

      ParseEstPlannerDebugToProto(
          std::move(turn_signal_res), est_result->est_status_list[plan_idx],
          std::move(est_result->scheduler_output_list[plan_idx]),
          std::move(est_result->est_planner_debug_list[plan_idx]),
          debug_proto.add_est_planner_debugs());
    }
    if (est_status.ok()) {
      debug_proto.set_turn_signal_reason_enum(
          debug_proto.est_planner_debugs()[0].turn_signal_reason_enum());
    }
  }
  if (!tl_info_collector_output.tl_info_map.empty()) {
    // Report candidate traffic light info.
    ReportCandidateTrafficLightInfo(tl_info_collector_output.tl_info_map,
                                    &debug_proto);
    // Report selected traffic light info.
    ReportSelectedTrafficLightInfo(
        tl_info_collector_output.tl_info_map,
        est_status.ok() ? planner_state->prev_target_lane_path
                        : mapping::LanePath(),
        debug_proto.mutable_selected_traffic_light_info());
  }
  *debug_proto.mutable_speed_considered_objects_prediction() =
      std::move(est_result->speed_considered_objects_prediction);

  // -----------------------------------------------------------------
  // ---------------------- Fill charts data- ------------------------
  // -----------------------------------------------------------------
  if (IsNonEmptyPlannerResult(est_status)) {
    for (int plan_idx = 0; plan_idx < est_result->chart_data_list.size();
         ++plan_idx) {
      *chart_data.add_est_chart_bundles() =
          std::move(est_result->chart_data_list[plan_idx]);
    }
  }
  if (fallback_status.ok()) {
    // Fallback charts are not to be switched.
    chart_data.mutable_charts()->MergeFrom(
        est_result->fallback_chart_data.charts());
  }

  // -------------------------------------------------------------------
  // --------------------- Update planner state ------------------------
  // -------------------------------------------------------------------
  if (est_status.ok()) {
    ParseEstPlannerOutputToPlannerState(
        std::move(est_result->est_planner_output_list[0]), planner_state);
  }

  // ------------------------------------------------------
  // ------------------ Report HMI content ----------------
  // ------------------------------------------------------
  HmiContentProto hmi_proto;
  ReportHmiContent(input.route_output->route_content_proto,
                   *input.planner_input->semantic_map_manager,
                   target_lane_path_from_current, &hmi_proto);

  on_road_plan_status.ToProto(debug_proto.mutable_planner_status());

  result->trajectory_info = std::move(trajectory_info);
  result->debug_info = std::move(debug_proto);
  result->chart_data = std::move(chart_data);
  result->hmi_content = std::move(hmi_proto);

  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(est_result);

  // NOTE: This condition is added by onboard infra team(mengchunlei);
  if (OnTestBenchForRsim()) {
    // On rsim test, ignore this error
    return OkPlannerStatus();
  }
  return on_road_plan_status;
}

}  // namespace qcraft::planner
