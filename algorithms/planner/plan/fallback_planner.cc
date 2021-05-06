#include "onboard/planner/plan/fallback_planner.h"

#include <utility>

#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/object/drive_passage_filter.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/trajectory_validation.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

DEFINE_bool(validate_fallback_planner, true,
            "Whether to validate fallback planner trajectory. If false, the "
            "fallback trajectory would be always considered as a valid plan. "
            "Turn off this flag only in demo mode.");

namespace qcraft::planner {

PlannerStatus RunFallbackPlanner(
    const FallbackPlannerInput& input, const VehicleParamApi& vehicle_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const DecisionConstraintConfigProto& decision_constraint_config,
    const FallbackPlannerParamsProto& fallback_planner_params,
    FallbackPlannerOutput* output,
    vis::vantage::ChartDataBundleProto* chart_data,
    FallbackPlannerDebugProto* debug, ThreadPool* thread_pool) {
  SCOPED_QTRACE("FallbackPlan");

  // Input sanity checks.
  QCHECK_NOTNULL(input.psmm);
  QCHECK_NOTNULL(input.pose);
  QCHECK_NOTNULL(input.start_point_info);
  QCHECK_NOTNULL(input.time_aligned_prev_trajectory);
  QCHECK_NOTNULL(input.prev_target_lane_path_from_start);
  QCHECK_NOTNULL(input.prev_lane_path_before_lc);
  QCHECK_NOTNULL(input.route_sections_info_from_start);
  QCHECK_NOTNULL(input.obj_mgr);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.stalled_objects);
  QCHECK_NOTNULL(input.scene_reasoning);
  QCHECK_NOTNULL(input.prev_lc_state);
  QCHECK_NOTNULL(input.traffic_light_states);
  QCHECK_NOTNULL(input.pre_decider_state);
  QCHECK_NOTNULL(input.tl_info_map);
  QCHECK_NOTNULL(input.smooth_result_map);

  const auto& time_aligned_prev_trajectory =
      *input.time_aligned_prev_trajectory;
  if (time_aligned_prev_trajectory.size() != kTrajectorySteps) {
    return PlannerStatus(PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
                         "Previous trajectory not available.");
  }
  if (input.prev_target_lane_path_from_start->IsEmpty()) {
    return PlannerStatus(PlannerStatusProto::FALLBACK_PREVIOUS_INFO_UNAVAILABLE,
                         "Previous target lane path not available.");
  }

  const auto& plan_start_point = input.start_point_info->start_point;
  const auto& psmm = *input.psmm;
  const auto vehicle_geometry_params = vehicle_params.vehicle_geometry_params();
  const auto vehicle_drive_params = vehicle_params.vehicle_drive_params();

  // Here we assume the drive passage's center stations are all virtual to build
  // a wide enough path boundary to contain the previous-trajectory path.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto drive_passage,
      BuildDrivePassage(
          psmm, *input.prev_target_lane_path_from_start, *input.station_anchor,
          input.route_sections_info_from_start->planning_horizon(),
          kDrivePassageKeepBehindLength, /*all_lanes_virtual=*/true),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto path_sl_boundary,
      BuildPathBoundaryFromPose(psmm, drive_passage, plan_start_point,
                                vehicle_geometry_params, *input.st_traj_mgr,
                                *input.prev_lc_state, *input.smooth_result_map,
                                /*borrow=*/false, input.prev_smooth_state),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  // Extend previous trajectory to a path that has a minimum length.
  const double min_fallback_path_length = 10.0;  // m.
  const double max_curvature =
      GetCenterMaxCurvature(vehicle_geometry_params, vehicle_drive_params);
  const auto path_extension_output = ExtendPathAndDeleteUnreasonablePart(
      time_aligned_prev_trajectory, &drive_passage, &path_sl_boundary,
      min_fallback_path_length, max_curvature);
  if (!path_extension_output.ok()) {
    return PlannerStatus(
        PlannerStatusProto::PATH_EXTENSION_FAILED,
        absl::StrCat("Path extension failed: ",
                     path_extension_output.status().message()));
  }

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto ego_frenet_box,
      drive_passage.QueryFrenetBoxAt(GetAvBox(
          Vec2dFromApolloTrajectoryPointProto(plan_start_point),
          plan_start_point.path_point().theta(), vehicle_geometry_params)),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);

  // Fake a scheduler output for fallback planner, to be moved later.
  auto scheduler_output = SchedulerOutput{
      .is_fallback = true,
      .drive_passage = drive_passage,
      .sl_boundary = path_sl_boundary,
      .lane_change_state = *input.prev_lc_state,
      .lane_path_before_lc = *input.prev_lane_path_before_lc,
      .length_along_route = input.prev_length_along_route,
      .av_frenet_box_on_drive_passage = std::move(ego_frenet_box)};

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

  // Build constraint manager. Fallback planner and Est planner share the same
  // set of decision constraints.
  // TODO(yumeng): Serialize this decision context if it need to be persist
  // across iterations.
  // We don't have clearance check output for previous trajectory but it is ok
  // to set it to null because it will be eventually discarded in the future.
  std::optional<ClearanceCheckOutput> clearance_output = std::nullopt;
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geometry_params,
      .config = &decision_constraint_config,
      .planner_semantic_map_manager = &psmm,
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .lc_stage = input.prev_lc_state->stage(),
      .plan_start_point = &plan_start_point,
      .lane_path_before_lc = input.prev_lane_path_before_lc,
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
      .vehicle_model = vehicle_params.vehicle_params().model(),
      .plan_time = input.start_point_info->plan_time,
      .sensor_fovs = input.sensor_fovs};

  // To be moved later.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);

  SpeedFinderInput speed_input{
      .base_name = "fallback",
      .psmm = &psmm,
      .traj_mgr = st_traj_mgr.get(),
      .constraint_mgr = &decider_output.constraint_manager,
      .drive_passage = &drive_passage,
      .route_sections_from_start =
          input.route_sections_info_from_start->route_sections(),
      .path_sl_boundary = &path_sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &*path_extension_output,
      .plan_start_point = plan_start_point};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geometry_params, vehicle_drive_params,
                motion_constraint_params,
                fallback_planner_params.speed_finder_params(), thread_pool),
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

  if (FLAGS_validate_fallback_planner &&
      !ValidateEstTrajectory(
          *psmm.semantic_map_manager(), speed_output.considered_st_objects,
          input.start_point_info->full_stop, *input.pose, scheduler_output,
          vehicle_geometry_params, vehicle_drive_params,
          motion_constraint_params, speed_output.trajectory_points,
          debug->mutable_traj_validation(), thread_pool)) {
    return PlannerStatus(PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
                         absl::StrCat("Validation failed: ",
                                      debug->traj_validation().DebugString()));
  }

  output->decider_state = std::move(decider_output.decider_state);
  output->trajectory_points = std::move(speed_output.trajectory_points);
  output->filtered_traj_mgr = std::move(st_traj_mgr);
  output->considered_st_objects = std::move(speed_output.considered_st_objects);
  output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  output->scheduler_output = std::move(scheduler_output);

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
