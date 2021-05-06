#include "onboard/planner/plan/expert_planner.h"

#include <utility>

#include "offboard/planner/ml/datasets/pnc_scenario_dataset/proto/pnc_scenario.pb.h"
#include "offboard/planner/ml/datasets/pnc_scenario_dataset/proto_converter.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/object/drive_passage_filter.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/multi_tasks_scheduler.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/target_lane_path_filter.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/trajectory_validation.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GetExpertTrajectoryFromFile(const std::string expert_trajectory_file_path,
                            const absl::Time plan_start_time) {
  // Read local file to load expert future trajectory
  planning_dataset::Scenario scenario;
  if (!file_util::BinaryFileToProto(expert_trajectory_file_path, &scenario)) {
    return absl::FailedPreconditionError(
        absl::StrFormat("Fail to read expert_trajectory_file at %s",
                        expert_trajectory_file_path));
  }

  // Load expert future trajectory into history buffer for later interpolation
  const auto& expert_future_trajectory =
      scenario.tracks(scenario.ego_track_index());
  HistoryBuffer<planning_dataset::ObjectState> buffer;
  for (int i = 0, size = scenario.timestamps_seconds_size(); i < size; ++i) {
    buffer.push_back(scenario.timestamps_seconds(i),
                     expert_future_trajectory.states(i));
  }

  // Interpolate the trajectory from plan_time with kTrajectoryTimeStep
  std::vector<planning_dataset::ObjectState> objs;
  const double planner_start_timestamp_sec =
      ToUnixDoubleSeconds(plan_start_time);
  auto interpolated_buffer = HistoryBuffer<planning_dataset::ObjectState>();
  for (int i = 0; i < kTrajectorySteps; ++i) {
    const double t = planner_start_timestamp_sec + i * kTrajectoryTimeStep;
    const int least_idx = buffer.GetIndexWithTimeAtLeast(t);
    const int most_idx = buffer.GetIndexWithTimeAtMost(t);
    if (least_idx == buffer.size() || most_idx == -1) {
      return absl::FailedPreconditionError(absl::StrFormat(
          "Interpolation t %f outside of collected ego future time range [%f, "
          "%f]",
          t, buffer.front_time(), buffer.back_time()));
    }
    planning_dataset::ObjectState object_state;
    ObjectStateLinearInterpolation(
        buffer[most_idx].second, buffer[least_idx].second,
        buffer[most_idx].first, buffer[least_idx].first, t, &object_state);
    interpolated_buffer.push_back(t, object_state);
  }

  auto trajectory_or = ObjectStatesToApolloTrajectory(interpolated_buffer);
  if (!trajectory_or.ok()) {
    return absl::FailedPreconditionError(
        "Fail to read convert a expert trajectory point state to "
        "ApolloTrajectoryProto.");
  }
  return *trajectory_or;
}

bool IsDrivePassageValid(
    const std::vector<ApolloTrajectoryPointProto>& trajectory,
    const DrivePassage& drive_passage) {
  for (const auto& point : trajectory) {
    const auto pos = Vec2dFromApolloTrajectoryPointProto(point);
    auto offset_or = drive_passage.QueryCurbOffsetAt(pos);
    if (!offset_or.ok()) {
      return false;
    }
    auto l_or = drive_passage.QueryFrenetLatOffsetAt(pos);
    if (!l_or.ok()) {
      return false;
    }

    if (*l_or < offset_or->first || *l_or > offset_or->second) {
      return false;
    }
  }
  return true;
}

bool IsPathSlBoundaryValid(
    const std::vector<ApolloTrajectoryPointProto>& trajectory,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary) {
  for (const auto& point : trajectory) {
    const auto pos = Vec2dFromApolloTrajectoryPointProto(point);
    auto l_or = drive_passage.QueryFrenetLatOffsetAt(pos);
    if (!l_or.ok()) {
      return false;
    }
    auto s = drive_passage.FindNearestStation(pos).accumulated_s();
    auto offset = path_sl_boundary.QueryBoundaryL(s);
    if (*l_or < offset.first || *l_or > offset.second) {
      return false;
    }
  }
  return true;
}

PlannerStatus RunExpertPlanner(const ExpertPlannerInput& input,
                               ExpertPlannerOutput* output,
                               vis::vantage::ChartDataBundleProto* chart_data,
                               ExpertPlannerDebugProto* debug,
                               ThreadPool* thread_pool) {
  SCOPED_QTRACE("ExpertPlan");

  // Input sanity checks.
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.pose);
  QCHECK_NOTNULL(input.start_point_info);
  QCHECK_NOTNULL(input.prev_target_lane_path_from_start);
  QCHECK_NOTNULL(input.sections_info_from_start);
  QCHECK_NOTNULL(input.obj_mgr);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.stalled_objects);
  QCHECK_NOTNULL(input.scene_reasoning);
  QCHECK_NOTNULL(input.traffic_light_states);
  QCHECK_NOTNULL(input.pre_decider_state);
  QCHECK_NOTNULL(input.tl_info_map);
  QCHECK_NOTNULL(input.lp_infos);
  QCHECK_NOTNULL(input.planner_params);
  QCHECK_NOTNULL(input.vehicle_params);
  QCHECK_NOTNULL(input.station_anchor);
  QCHECK_NOTNULL(input.smooth_result_map);

  // Load expert trajectory from file.
  auto trajectory_or = GetExpertTrajectoryFromFile(
      FLAGS_expert_trajectory_file_path, input.start_point_info->plan_time);
  if (!trajectory_or.ok()) {
    return PlannerStatus(PlannerStatusProto::EXPERT_TRAJ_UNAVAILABLE,
                         "Expert traj reading failed.");
  }
  const auto& plan_start_point = input.start_point_info->start_point;
  const auto& psmm = *input.psmm;
  const auto& vehicle_geometry_params =
      input.vehicle_params->vehicle_geometry_params();
  const auto& vehicle_drive_params =
      input.vehicle_params->vehicle_drive_params();

  const auto target_lane_path_info =
      ChooseLeastLateralOffsetLanePath(*input.lp_infos, *trajectory_or);
  const auto target_lane_path = target_lane_path_info.lane_path();

  auto drive_passage_or = BuildDrivePassage(
      *input.psmm, target_lane_path, *input.station_anchor,
      input.sections_info_from_start->planning_horizon(),
      kDrivePassageKeepBehindLength, FLAGS_planner_consider_all_lanes_virtual);
  if (!drive_passage_or.ok()) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        "Drive passage recontruction for expert trajectory failed.");
  }

  if (!IsDrivePassageValid(*trajectory_or, *drive_passage_or)) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
        "Drive passage not valid for expert trajectory.");
  }

  const bool should_smooth = ShouldSmoothRefLane(
      *input.tl_info_map, *drive_passage_or, input.prev_smooth_state);

  auto no_borrow_scheduler_output_or = MakeSchedulerOutput(
      *input.psmm, *input.sections_info_from_start, *input.lp_infos,
      *drive_passage_or, target_lane_path_info, vehicle_geometry_params,
      *input.planner_params, *input.st_traj_mgr, *input.stalled_objects,
      input.start_point_info->start_point,
      *input.prev_target_lane_path_from_start, *input.smooth_result_map,
      /*borrow=*/false, should_smooth, thread_pool);

  SchedulerOutput scheduler_output;

  if (no_borrow_scheduler_output_or.ok() &&
      IsPathSlBoundaryValid(*trajectory_or, *drive_passage_or,
                            no_borrow_scheduler_output_or->sl_boundary)) {
    scheduler_output = *no_borrow_scheduler_output_or;
  } else {
    auto borrow_scheduler_output_or = MakeSchedulerOutput(
        *input.psmm, *input.sections_info_from_start, *input.lp_infos,
        *drive_passage_or, target_lane_path_info, vehicle_geometry_params,
        *input.planner_params, *input.st_traj_mgr, *input.stalled_objects,
        input.start_point_info->start_point,
        *input.prev_target_lane_path_from_start, *input.smooth_result_map,
        /*borrow=*/true, should_smooth, thread_pool);
    if (borrow_scheduler_output_or.ok() &&
        IsPathSlBoundaryValid(*trajectory_or, *drive_passage_or,
                              borrow_scheduler_output_or->sl_boundary)) {
      scheduler_output = *borrow_scheduler_output_or;
    } else {
      return PlannerStatus(
          PlannerStatusProto::EXPERT_TRAJ_INTERMEDIATES_RECONSTRUCTION_FAILED,
          "Scheduler output recontruction for expert trajectory failed.");
    }
  }

  const auto& drive_passage = scheduler_output.drive_passage;
  const auto& path_sl_boundary = scheduler_output.sl_boundary;
  scheduler_output.is_expert = true;

  // Filter the space-time object trajectories.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      FLAGS_planner_only_use_most_likely_trajectory);
  const DrivePassageFilter drive_passage_filter(&drive_passage,
                                                &path_sl_boundary);
  auto st_traj_mgr = std::make_unique<SpacetimeTrajectoryManager>(
      absl::Span<const TrajectoryFilter* const>(
          {&low_likelihood_filter, &drive_passage_filter}),
      absl::Span<const std::unique_ptr<SpacetimePlannerTrajectoryFinder>>(),
      input.obj_mgr->planner_objects(),
      /*st_planner_trajectories_start_offset=*/0.0, thread_pool);

  // Build constraint manager.
  std::optional<ClearanceCheckOutput> clearance_output = std::nullopt;
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geometry_params,
      .config = &input.planner_params->decision_constraint_config(),
      .planner_semantic_map_manager = &psmm,
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .lc_stage = scheduler_output.lane_change_state.stage(),
      .plan_start_point = &plan_start_point,
      .lane_path_before_lc = &scheduler_output.lane_path_before_lc,
      .passage = &drive_passage,
      .sl_boundary = &path_sl_boundary,
      .obj_mgr = input.obj_mgr,
      .st_traj_mgr = st_traj_mgr.get(),
      .tl_info_map = input.tl_info_map,
      .lc_clearance_check_output = &clearance_output,
      .pre_decider_state = input.pre_decider_state,
      .pose = input.pose,
      .av_frenet_box = &scheduler_output.av_frenet_box_on_drive_passage,
      .parking_brake_release_time = input.parking_brake_release_time,
      .teleop_enable_traffic_light_stop =
          input.teleop_enable_traffic_light_stop,
      .enable_pull_over = input.enable_pull_over,
      .brake_to_stop = input.brake_to_stop,
      .length_along_route = scheduler_output.length_along_route,
      .vehicle_model = input.vehicle_params->vehicle_params().model(),
      .plan_time = input.start_point_info->plan_time,
      .sensor_fovs = input.sensor_fovs};

  // To be moved later.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);

  // TODO(Jinyun): Skip path extention when speed is too low.
  constexpr double kRequiredMinPathLength = 10.0;
  const double max_curvature =
      GetCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  const auto path_extension_output = ExtendPathAndDeleteUnreasonablePart(
      *trajectory_or, &drive_passage, &path_sl_boundary,
      /*required_min_length=*/kRequiredMinPathLength, max_curvature);
  if (!path_extension_output.ok()) {
    return PlannerStatus(
        PlannerStatusProto::PATH_EXTENSION_FAILED,
        absl::StrCat("Path extension failed: ",
                     path_extension_output.status().message()));
  }

  SpeedFinderInput speed_input{
      .base_name = "expert_speed_recalculated",
      .psmm = &psmm,
      .traj_mgr = st_traj_mgr.get(),
      .constraint_mgr = &decider_output.constraint_manager,
      .drive_passage = &drive_passage,
      .route_sections_from_start =
          input.sections_info_from_start->route_sections(),
      .path_sl_boundary = &path_sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &*path_extension_output,
      .plan_start_point = plan_start_point};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geometry_params, vehicle_drive_params,
                input.planner_params->motion_constraint_params(),
                input.planner_params->speed_finder_params(), thread_pool),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);

  // Send charts.
  chart_data->add_charts()->Swap(&speed_output.st_graph_chart);
  chart_data->add_charts()->Swap(&speed_output.vt_graph_chart);
  chart_data->add_charts()->Swap(&speed_output.traj_chart);
  if (FLAGS_planner_send_speed_path_chart_data) {
    chart_data->add_charts()->Swap(&speed_output.path_chart);
  }

  // Send debug info.
  ToSchedulerOutputProto(scheduler_output, debug->mutable_scheduler());

  UpdateDecisionConstraintDebugInfo(speed_output.constraint_mgr,
                                    debug->mutable_constraint());
  *debug->mutable_speed_finder() = std::move(speed_output.speed_finder_proto);

  if (!ValidateEstTrajectory(
          *psmm.semantic_map_manager(), speed_output.considered_st_objects,
          input.start_point_info->full_stop, *input.pose, scheduler_output,
          vehicle_geometry_params, vehicle_drive_params,
          input.planner_params->motion_constraint_params(),
          speed_output.trajectory_points, debug->mutable_traj_validation(),
          thread_pool)) {
    return PlannerStatus(
        PlannerStatusProto::EXPERT_SPEED_IN_CONTRADICTORY_WITH_SPEED_FINDER,
        absl::StrCat("Validation failed: ",
                     debug->traj_validation().DebugString()));
  }

  output->decider_state = std::move(decider_output.decider_state);
  // Output the original trajectory points here.
  output->trajectory_points = std::move(*trajectory_or);
  output->filtered_traj_mgr = std::move(st_traj_mgr);
  output->considered_st_objects = std::move(speed_output.considered_st_objects);
  output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  output->scheduler_output = std::move(scheduler_output);

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
