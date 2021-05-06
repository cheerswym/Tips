#include "onboard/planner/plan/multi_tasks_async_planner.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "gflags/gflags_declare.h"
#include "glog/logging.h"
#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/async/future.h"
#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/plan/fallback_planner.h"
#include "onboard/planner/plan/st_path_planner.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/planner_state.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/route.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/multi_tasks_scheduler.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_input.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/scheduler/target_lane_path_filter.h"
#include "onboard/planner/selector/selector.h"
#include "onboard/planner/selector/selector_input.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/planner/trajectory_end_info.h"
#include "onboard/planner/trajectory_validation.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace {

void AppendFallbackToResultList(const PlannerStatus &fallback_status,
                                FallbackPlannerOutput fallback_result,
                                std::vector<SchedulerOutput> *multi_tasks,
                                std::vector<PlannerStatus> *status_list,
                                std::vector<EstPlannerOutput> *results) {
  status_list->push_back(fallback_status);
  multi_tasks->push_back(std::move(fallback_result.scheduler_output));
  results->emplace_back(EstPlannerOutput{
      .traj_points = std::move(fallback_result.trajectory_points),
      .decider_state = std::move(fallback_result.decider_state),
      .filtered_traj_mgr = std::move(fallback_result.filtered_traj_mgr),
      .considered_st_objects = std::move(fallback_result.considered_st_objects),
      .trajectory_end_info = std::move(fallback_result.trajectory_end_info)});
}

absl::StatusOr<mapping::LanePath> AlignLanePathWithRouteSections(
    const RouteSectionsInfo &sections_info, const mapping::LanePath &lane_path,
    double proj_range) {
  const auto &front_id_map = sections_info.front().id_idx_map;
  for (const auto &lane_seg : lane_path.BeforeArclength(proj_range)) {
    if (front_id_map.contains(lane_seg.lane_id)) {
      return lane_path.AfterFirstOccurrenceOfLanePoint(mapping::LanePoint(
          lane_seg.lane_id, std::max(lane_seg.start_fraction,
                                     sections_info.front().start_fraction)));
    }
  }
  return absl::OutOfRangeError(
      "Prev target lane path has no overlap with the current route sections.");
}

struct SchedulerAndPathInput {
  const VehicleGeometryParamsProto *vehicle_geom_params = nullptr;
  const VehicleDriveParamsProto *vehicle_drive_params = nullptr;
  const PlannerSemanticMapManager *planner_semantic_map_manager = nullptr;
  const PlannerParamsProto *planner_params = nullptr;
  const PoseProto *pose = nullptr;
  VehicleModel vehicle_model;
  const StPathPlanStartPointInfo *start_point_info = nullptr;
  const RouteSections *route_sections_from_start = nullptr;
  const absl::flat_hash_set<mapping::ElementId> *avoid_lanes = nullptr;
  const PlannerObjectManager *obj_mgr = nullptr;
  bool prev_smooth_state = false;
  const absl::flat_hash_set<std::string> *stalled_objects = nullptr;
  const std::vector<ApolloTrajectoryPointProto> *time_aligned_prev_traj =
      nullptr;
  const mapping::LanePath *prev_target_lane_path = nullptr;
  const mapping::LanePath *preferred_lane_path = nullptr;
  const TrafficLightInfoMap *tl_info_map = nullptr;
  const SmoothedReferenceLineResultMap *smooth_result_map = nullptr;
  const mapping::LanePoint *station_anchor = nullptr;
  const SceneOutputProto *scene_reasoning = nullptr;
  const SensorFovsProto *sensor_fovs = nullptr;
  const TeleopState *teleop_state = nullptr;
  const SpacetimePlannerTrajectories *prev_st_planner_trajectories = nullptr;
  const DeciderStateProto *prev_decider_state = nullptr;
  const InitializerStateProto *prev_initializer_state = nullptr;
  absl::Time parking_brake_release_time;
  const TrajectoryProto *previous_trajectory = nullptr;
};

std::unique_ptr<AsyncPathOutput> RunSchedulerAndPathPlanner(
    const SchedulerAndPathInput &input, ThreadPool *thread_pool) {
  auto output = std::make_unique<AsyncPathOutput>();

  output->async_path_status = OkPlannerStatus();
  output->route_sections_from_start = *input.route_sections_from_start;
  output->avoid_lanes = *input.avoid_lanes;

  const auto *psmm = input.planner_semantic_map_manager;
  const RouteSectionsInfo route_sections_info_from_start(
      *psmm, &output->route_sections_from_start, output->avoid_lanes);
  // --------------------------------------------------------------
  // ----------------- Choose candidates on lane graph ------------
  // --------------------------------------------------------------
  auto lane_graph = std::make_unique<LaneGraph>(
      BuildLaneGraph(*psmm, route_sections_info_from_start, *input.obj_mgr,
                     *input.stalled_objects, output->avoid_lanes));

  if (FLAGS_planner_send_lane_graph_to_canvas) {
    SendLaneGraphToCanvas(*lane_graph, *psmm->semantic_map_manager(),
                          route_sections_info_from_start, "planner/lane_graph");
  }

  // Collect candidate lane paths from each start lane. If one or more
  // diverging point exists starting from some lane, the first one would be
  // considered, thus two candidates would be produced.
  output->lp_infos = FindBestLanePathsFromStart(
      *psmm, route_sections_info_from_start, *lane_graph, thread_pool);
  if (output->lp_infos.empty()) {
    // In case all lane paths are blocked by stalled objects.
    output->async_path_status =
        PlannerStatus(PlannerStatusProto::PLANNER_ABNORMAL_EXIT,
                      "No viable candidate route found to destination.");
    return output;
  }
  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(lane_graph);

  // ----------------------------------------------------------
  // ---------------- Project path_plan_start_point ----------------
  // ----------------------------------------------------------
  // Guarantee that the lane path starts from the current section.
  if (auto status_or = AlignLanePathWithRouteSections(
          route_sections_info_from_start, *input.prev_target_lane_path,
          kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength);
      status_or.ok()) {
    output->prev_target_lane_path_from_start = std::move(status_or).value();
  } else {
    output->async_path_status = PlannerStatus(
        PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED,
        status_or.status().ToString());
    return output;
  }

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      route_sections_info_from_start, output->prev_target_lane_path_from_start,
      input.start_point_info->start_point, *input.preferred_lane_path,
      &output->lp_infos);

  // --------------------------------------------------------------
  // ----------------- Run Scheduler ------------------------------
  // --------------------------------------------------------------
  auto st_traj_mgr = std::make_unique<SpacetimeTrajectoryManager>(
      absl::Span<const TrajectoryFilter *>{},
      absl::Span<const std::unique_ptr<SpacetimePlannerTrajectoryFinder>>{},
      input.obj_mgr->planner_objects(),
      /*st_planner_trajectories_start_offset=*/0.0, thread_pool);
  const auto &planner_params = *input.planner_params;
  MultiTasksSchedulerInput scheduler_input{
      .psmm = psmm,
      .vehicle_geom = input.vehicle_geom_params,
      .planner_params = &planner_params,
      .st_traj_mgr = st_traj_mgr.get(),
      .stalled_objects = input.stalled_objects,
      .lane_path_infos = &output->lp_infos,
      .sections_info_from_current = &route_sections_info_from_start,
      .tl_info_map = input.tl_info_map,
      .prev_smooth_state = input.prev_smooth_state,
      .plan_start_point = &input.start_point_info->start_point,
      .station_anchor = input.station_anchor,
      .start_route_s = 0.0,
      .prev_lane_path_from_current = &output->prev_target_lane_path_from_start,
      .smooth_result_map = input.smooth_result_map,
      .thread_pool = thread_pool};
  const absl::Cleanup clean_object_container_async = [&st_traj_mgr] {
    MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(st_traj_mgr);
  };

  if (auto status_or = ScheduleMultiplePlanTasks(scheduler_input,
                                                 target_lp_infos, thread_pool);
      status_or.ok()) {
    output->scheduler_output = std::move(status_or).value();
  } else {
    output->async_path_status =
        PlannerStatus(PlannerStatusProto::SCHEDULER_UNAVAILABLE,
                      status_or.status().ToString());
  }

  const int num_tasks = output->scheduler_output.size();
  output->path_infos.resize(num_tasks);

  ParallelFor(0, num_tasks, [&](int i) {
    const StPathPlannerInput st_path_input{
        .plan_id = i,
        .start_point_info = input.start_point_info,
        .scheduler_output = &output->scheduler_output[i],
        .vehicle_geom_params = input.vehicle_geom_params,
        .vehicle_drive_params = input.vehicle_drive_params,
        .planner_params = &planner_params,
        .obj_mgr = input.obj_mgr,
        .planner_semantic_map_manager = psmm,
        .stalled_objects = input.stalled_objects,
        .scene_reasoning = input.scene_reasoning,
        .tl_info_map = input.tl_info_map,
        .pose = input.pose,
        .teleop_state = input.teleop_state,
        .vehicle_model = &input.vehicle_model,
        .time_aligned_prev_traj = input.time_aligned_prev_traj,
        .sensor_fovs = input.sensor_fovs,
        .prev_st_planner_trajectories = input.prev_st_planner_trajectories,
        .prev_decider_state = input.prev_decider_state,
        .prev_initializer_state = input.prev_initializer_state,
        .parking_brake_release_time = input.parking_brake_release_time,
    };

    output->path_infos[i].path_status = RunStPathPlanner(
        st_path_input, &output->path_infos[i].path_output, thread_pool);
  });
  for (int i = 0; i < num_tasks; ++i) {
    if (!output->path_infos[i].path_status.ok()) {
      QLOG(WARNING) << "Failed task " << i << ": "
                    << output->path_infos[i].path_status.ToString();
    }
  }
  return output;
}

Future<std::shared_ptr<AsyncPathOutput>> ScheduleFuturePath(
    const MultiTasksEstPlannerInput &input, ThreadPool *thread_pool) {
  const auto &vehicle_params = input.planner_input->vehicle_params;
  const auto &vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const auto &planner_params = input.planner_params->planner_params();

  const auto path_start_point_info = GetStPathPlanStartPointInfo(
      absl::Milliseconds(FLAGS_planner_deferred_path_planning_time_ms),
      *input.start_point_info, input.planner_state->previous_trajectory);
  const auto *psmm = input.planner_input->planner_semantic_map_manager.get();
  return ScheduleFuture(
      thread_pool,
      [thread_pool, vehicle_geom_params, vehicle_drive_params,
       vehicle_model = vehicle_params.vehicle_params().model(), psmm,
       &planner_params, pose = *input.planner_input->pose,
       path_start_point_info,
       route_sections_from_start = *input.route_sections_from_start,
       avoid_lanes = input.route->avoid_lanes(),
       // NOTE(lidong): MUST reconstruct obj_mgr as it has internal pointers.
       obj_mgr = PlannerObjectManager(input.object_manager->planner_objects()),
       prev_smooth_state = input.prev_smooth_state,
       stalled_objects = *input.stalled_objects,
       time_aligned_prev_traj = *input.time_aligned_prev_traj,
       prev_target_lane_path = *input.prev_target_lane_path,
       preferred_lane_path = input.planner_state->preferred_lane_path,
       tl_info_map = *input.tl_info_map,
       smooth_result_map = *input.smooth_result_map,
       station_anchor = *input.station_anchor,
       scene_reasoning = *input.scene_reasoning,
       sensor_fovs = *input.sensor_fovs, teleop_state = *input.teleop_state,
       prev_st_planner_trajectories =
           input.planner_state->st_planner_trajectories,
       prev_decider_state = input.planner_state->decider_state,
       prev_initializer_state = input.planner_state->initializer_state,
       parking_brake_release_time =
           input.planner_state->parking_brake_release_time]() {
        const SchedulerAndPathInput scheduler_and_path_input{
            .vehicle_geom_params = &vehicle_geom_params,
            .vehicle_drive_params = &vehicle_drive_params,
            .planner_semantic_map_manager = psmm,
            .planner_params = &planner_params,
            .pose = &pose,
            .vehicle_model = vehicle_model,
            .start_point_info = &path_start_point_info,
            .route_sections_from_start = &route_sections_from_start,
            .avoid_lanes = &avoid_lanes,
            .obj_mgr = &obj_mgr,
            .prev_smooth_state = prev_smooth_state,
            .stalled_objects = &stalled_objects,
            .time_aligned_prev_traj = &time_aligned_prev_traj,
            .prev_target_lane_path = &prev_target_lane_path,
            .preferred_lane_path = &preferred_lane_path,
            .tl_info_map = &tl_info_map,
            .smooth_result_map = &smooth_result_map,
            .station_anchor = &station_anchor,
            .scene_reasoning = &scene_reasoning,
            .sensor_fovs = &sensor_fovs,
            .teleop_state = &teleop_state,
            .prev_st_planner_trajectories = &prev_st_planner_trajectories,
            .prev_decider_state = &prev_decider_state,
            .prev_initializer_state = &prev_initializer_state,
            .parking_brake_release_time = parking_brake_release_time};
        return std::shared_ptr<AsyncPathOutput>(
            RunSchedulerAndPathPlanner(scheduler_and_path_input, thread_pool));
      });
}

PlannerStatus RunSpeedPlanner(const MultiTasksEstPlannerInput &input,
                              const AsyncPathOutput &async_path_output,
                              int path_index, EstPlannerOutput *est_output,
                              ThreadPool *thread_pool) {
  const auto &scheduler_output = async_path_output.scheduler_output[path_index];
  const auto &path_info = async_path_output.path_infos[path_index];
  if (!path_info.path_status.ok()) {
    return path_info.path_status;
  }
  const auto &vehicle_params = input.planner_input->vehicle_params;
  const auto &vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto &vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const auto &psmm = *input.planner_input->planner_semantic_map_manager;
  const StPathPlannerOutput &path_output = path_info.path_output;
  const auto &drive_passage = scheduler_output.drive_passage;
  const auto &sl_boundary = scheduler_output.sl_boundary;
  const auto &planner_params = input.planner_params->planner_params();

  // Create trajectory manager for speed.
  SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
      .passage = &drive_passage,
      .sl_boundary = &sl_boundary,
      .obj_mgr = input.object_manager,
      .veh_geom = &vehicle_geom_params,
      .plan_start_point = &input.start_point_info->start_point,
      .st_planner_trajectories_start_index = 0,
      .lane_change_state = &scheduler_output.lane_change_state,
      .prev_st_trajs = &input.planner_state->st_planner_trajectories};
  auto traj_mgr =
      BuildSpacetimeTrajectoryManager(st_mgr_builder_input, thread_pool);

  const VehicleModel vehicle_model = vehicle_params.vehicle_params().model();
  // Create decision constraints for speed.
  DeciderInput decider_input{
      .vehicle_geometry_params = &vehicle_geom_params,
      .config = &planner_params.decision_constraint_config(),
      .planner_semantic_map_manager = &psmm,
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .lc_stage = scheduler_output.lane_change_state.stage(),
      .plan_start_point = &input.start_point_info->start_point,
      .lane_path_before_lc = &scheduler_output.lane_path_before_lc,
      .passage = &drive_passage,
      .sl_boundary = &sl_boundary,
      .borrow_lane_boundary = scheduler_output.borrow_lane,
      .obj_mgr = input.object_manager,
      .st_traj_mgr = traj_mgr.get(),
      .tl_info_map = input.tl_info_map,
      .lc_clearance_check_output = &scheduler_output.clearance_output,
      .pre_decider_state = &input.planner_state->decider_state,
      .pose = input.planner_input->pose.get(),
      .av_frenet_box = &scheduler_output.av_frenet_box_on_drive_passage,
      .parking_brake_release_time =
          input.planner_state->parking_brake_release_time,
      .teleop_enable_traffic_light_stop =
          input.teleop_state->enable_traffic_light_stopping,
      .enable_pull_over = input.teleop_state->enable_pull_over,
      .brake_to_stop = input.teleop_state->brake_to_stop,
      .length_along_route = scheduler_output.length_along_route,
      .vehicle_model = vehicle_model,
      .plan_time = input.start_point_info->plan_time,
      .sensor_fovs = input.sensor_fovs};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);

  // Update spacetime planner trajectory from leading obj.
  for (const auto &[traj_id, leading_object] :
       decider_output.constraint_manager.LeadingObjects()) {
    const auto status = traj_mgr->AddSpaceTimePlannerTrajectoryById(
        traj_id, SpacetimePlannerTrajectoryReason::LEADING);
    VLOG_IF(3, !status.ok()) << status.ToString();
  }

  // Run speed.
  const SpeedFinderInput speed_input{
      .base_name = "est",
      .psmm = &psmm,
      .traj_mgr = traj_mgr.get(),
      .constraint_mgr = &decider_output.constraint_manager,
      .drive_passage = &drive_passage,
      .route_sections_from_start = &async_path_output.route_sections_from_start,
      .path_sl_boundary = &sl_boundary,
      .stalled_objects = input.stalled_objects,
      .path = &path_output.path,
      // speed planning always starts at plan start point
      .plan_start_point = input.start_point_info->start_point,
  };

  const auto &motion_constraint_params =
      planner_params.motion_constraint_params();
  const auto &speed_finder_params = planner_params.speed_finder_params();

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto speed_output,
      FindSpeed(speed_input, vehicle_geom_params, vehicle_drive_params,
                motion_constraint_params, speed_finder_params, thread_pool),
      PlannerStatusProto::SPEED_OPTIMIZER_FAILED);
  est_output->chart_data.add_charts()->Swap(&speed_output.st_graph_chart);
  est_output->chart_data.add_charts()->Swap(&speed_output.vt_graph_chart);
  est_output->chart_data.add_charts()->Swap(&speed_output.traj_chart);
  if (FLAGS_planner_send_speed_path_chart_data) {
    est_output->chart_data.add_charts()->Swap(&speed_output.path_chart);
  }
  if (FLAGS_planner_send_interactive_speed_to_chart) {
    est_output->chart_data.add_charts()->Swap(
        &speed_output.interactive_speed_chart);
  }

  est_output->debug_info.speed_finder_debug.CopyFrom(
      speed_output.speed_finder_proto);

  UpdateDecisionConstraintDebugInfo(
      speed_output.constraint_mgr,
      &est_output->debug_info.decision_constraints);

  // Move final trajectory to output.
  est_output->traj_points = std::move(speed_output.trajectory_points);

  const bool valid = ValidateEstTrajectory(
      *psmm.semantic_map_manager(), speed_output.considered_st_objects,
      input.start_point_info->full_stop, *input.planner_input->pose,
      scheduler_output, vehicle_geom_params, vehicle_drive_params,
      motion_constraint_params, est_output->traj_points,
      &est_output->debug_info.traj_validation_result, thread_pool);
  if (!valid && FLAGS_est_fallback_upon_traj_validation_failure) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat(
            "Validation failed: ",
            est_output->debug_info.traj_validation_result.DebugString()));
  }

  est_output->chart_data.MergeFrom(path_output.chart_data);
  // Set cross-frame state of the decider.
  est_output->decider_state = decider_output.decider_state;
  // Set cross-frame state of the initializer.
  est_output->initializer_state = path_output.initializer_state;
  est_output->considered_st_objects =
      std::move(speed_output.considered_st_objects);
  est_output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  est_output->feature_costs = path_output.opt_feature_costs;
  est_output->expert_costs = path_output.opt_expert_costs;
  est_output->debug_info.optimizer_debug_proto =
      path_output.optimizer_debug_proto;
  est_output->debug_info.initializer_debug_proto =
      path_output.initializer_debug_proto;

  // Set cross-frame state of spacetime planner objects.
  for (const auto &st_planner_traj_info :
       traj_mgr->spacetime_planner_trajs_info()) {
    auto *st_planner_traj_proto =
        est_output->st_planner_trajectories.add_trajectory();
    st_planner_traj_proto->set_reason(st_planner_traj_info.reason);
    st_planner_traj_proto->set_id(st_planner_traj_info.object_id);
    st_planner_traj_proto->set_index(st_planner_traj_info.traj_index);
  }

  est_output->debug_info.st_planner_trajectories =
      est_output->st_planner_trajectories;

  // Export filtered object to debug message.
  est_output->debug_info.filtered_prediction_trajectories.mutable_filtered()
      ->Reserve(traj_mgr->ignored_trajectories().size());
  for (const auto &ignored : traj_mgr->ignored_trajectories()) {
    auto *filtered =
        est_output->debug_info.filtered_prediction_trajectories.add_filtered();
    filtered->set_reason(ignored.reason);
    filtered->set_id(ignored.object_id);
    filtered->set_index(ignored.traj->index());
  }
  est_output->filtered_traj_mgr = std::move(traj_mgr);

  return OkPlannerStatus();
}
}  // namespace

PlannerStatus RunMultiTasksAsyncPlanner(const MultiTasksEstPlannerInput &input,
                                        const AsyncPathOutput *prev_path,
                                        PathBoundedEstPlannerOutput *output,
                                        AsyncPathState *async_path_state,
                                        ThreadPool *thread_pool) {
  FUNC_QTRACE();

  const auto *psmm = input.planner_input->planner_semantic_map_manager.get();

  // ------------------------------------------------------------------
  // --------------------- Fallback planner ---------------------------
  // ------------------------------------------------------------------
  FallbackPlannerOutput fallback_result;
  Future<PlannerStatus> future_fallback_status =
      ScheduleFuture(thread_pool, [&]() {
        const RouteSectionsInfo route_sections_info_from_start(
            *psmm, input.route_sections_from_start, input.route->avoid_lanes());
        // Guarantee that the lane path starts from the current section.
        RETURN_PLANNER_STATUS_OR_ASSIGN(
            const auto prev_target_lane_path_from_start,
            AlignLanePathWithRouteSections(route_sections_info_from_start,
                                           *input.prev_target_lane_path,
                                           kMaxTravelDistanceBetweenFrames +
                                               kDrivePassageKeepBehindLength),
            PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);
        const FallbackPlannerInput fallback_input{
            .psmm = psmm,
            .pose = input.planner_input->pose.get(),
            .start_point_info = input.start_point_info,
            .time_aligned_prev_trajectory = input.time_aligned_prev_traj,
            .prev_target_lane_path_from_start =
                &prev_target_lane_path_from_start,
            .prev_length_along_route =
                input.planner_state->prev_length_along_route,
            .station_anchor = input.station_anchor,
            .prev_smooth_state = input.planner_state->prev_smooth_state,
            .prev_lane_path_before_lc =
                &input.planner_state->prev_lane_path_before_lc,
            .route_sections_info_from_start = &route_sections_info_from_start,
            .obj_mgr = input.object_manager,
            .st_traj_mgr = input.st_traj_mgr,
            .stalled_objects = input.stalled_objects,
            .scene_reasoning = input.scene_reasoning,
            .prev_lc_state = &input.planner_state->lane_change_state,
            .traffic_light_states =
                input.planner_input->traffic_light_states.get(),
            .pre_decider_state = &input.planner_state->decider_state,
            .tl_info_map = input.tl_info_map,
            .smooth_result_map = input.smooth_result_map,
            .parking_brake_release_time =
                input.planner_state->parking_brake_release_time,
            .teleop_enable_traffic_light_stop =
                input.teleop_state->enable_traffic_light_stopping,
            .enable_pull_over = input.teleop_state->enable_pull_over,
            .brake_to_stop = input.teleop_state->brake_to_stop,
            .sensor_fovs = input.planner_input->sensor_fovs.get()};

        return RunFallbackPlanner(
            fallback_input, input.planner_input->vehicle_params,
            input.planner_params->planner_params().motion_constraint_params(),
            input.planner_params->planner_params().decision_constraint_config(),
            input.planner_params->planner_params().fallback_planner_params(),
            &fallback_result, &output->fallback_chart_data,
            &output->fallback_debug, /*thread_pool=*/nullptr);
      });

  const absl::Cleanup wait_for_fallback = [&future_fallback_status, &output]() {
    if (future_fallback_status.IsValid()) {
      output->fallback_status = future_fallback_status.Get();
    }
  };

  // ---------------------------------------
  // ------ Run Async Path Planner ---------
  // ---------------------------------------
  const AsyncPathOutput *async_path_output = prev_path;
  if (async_path_output == nullptr) {  // If there is no previous path.
    const bool is_path_future_valid = async_path_state->path_future.IsValid();
    if (is_path_future_valid) {
      async_path_state->current_path = async_path_state->path_future.Get();
      if (async_path_state->current_path == nullptr) {
        return PlannerStatus(PlannerStatusProto::PATH_PLANNER_FAILED,
                             absl::StrCat("Async path planner failed."));
      }
      async_path_output = async_path_state->current_path.get();
    }
    // Schedule for computing path for future iterations.
    async_path_state->path_future = ScheduleFuturePath(input, thread_pool);

    if (!is_path_future_valid) {
      return PlannerStatus(PlannerStatusProto::PATH_PLANNER_FAILED,
                           "Invalid path future.");
    }
  }
  if (!QCHECK_NOTNULL(async_path_output)->async_path_status.ok()) {
    return PlannerStatus(PlannerStatusProto::PATH_PLANNER_FAILED,
                         async_path_output->async_path_status.ToString());
  }

  // ---------------------------------------
  // ------ Run Speed Planner ----------------
  // ---------------------------------------
  const int num_tasks = async_path_output->path_infos.size();
  std::vector<PlannerStatus> status_lists(num_tasks);
  std::vector<EstPlannerOutput> results(num_tasks);
  std::vector<EstPlannerDebug> est_debugs(num_tasks);
  std::vector<vis::vantage::ChartDataBundleProto> charts_datas(num_tasks);

  std::vector<SchedulerOutput> multi_tasks(num_tasks);
  ParallelFor(0, num_tasks, thread_pool, [&](int i) {
    status_lists[i] =
        RunSpeedPlanner(input, *async_path_output, i, &results[i], thread_pool);

    // Make a copy for selector to use.
    multi_tasks[i] = async_path_output->scheduler_output[i];

    est_debugs[i] = std::move(results[i].debug_info);
    charts_datas[i] = std::move(results[i].chart_data);
  });

  // Collect speed-considered object ids by all est planners and fallback
  // planner.
  absl::flat_hash_set<std::string> speed_considered_object_ids;
  for (int i = 0; i < num_tasks; ++i) {
    for (const auto &part_st_traj : results[i].considered_st_objects) {
      speed_considered_object_ids.insert(
          std::string(part_st_traj.st_traj().object_id()));
    }
    if (!status_lists[i].ok()) {
      QLOG(WARNING) << "Failed task " << i << ": " << status_lists[i].message();
    }
  }

  // Fill the fallback result to be selected together if:
  // - fallback planner succeeds, and
  // - no teleop-required lane change is activated, and
  //   - it matches the start id of some est planner, or
  //   - no est planner succeeds.
  output->fallback_status = future_fallback_status.Get();

  for (const auto &part_st_traj : fallback_result.considered_st_objects) {
    speed_considered_object_ids.insert(
        std::string(part_st_traj.st_traj().object_id()));
  }
  ObjectsPredictionProto speed_considered_objects_prediction;
  speed_considered_objects_prediction.mutable_objects()->Reserve(
      speed_considered_object_ids.size());
  for (const auto &object_id : speed_considered_object_ids) {
    const auto *object =
        QCHECK_NOTNULL(input.object_manager->FindObjectById(object_id));
    object->prediction().ToProto(
        speed_considered_objects_prediction.add_objects());
  }

  if (output->fallback_status.ok() &&
      input.planner_state->preferred_lane_path.IsEmpty()) {
    const bool est_any_success =
        std::any_of(status_lists.begin(), status_lists.end(),
                    [](const PlannerStatus &status) { return status.ok(); });
    if (!est_any_success) {
      AppendFallbackToResultList(output->fallback_status,
                                 std::move(fallback_result), &multi_tasks,
                                 &status_lists, &results);
    } else {
      const auto fallback_start_id =
          fallback_result.scheduler_output.drive_passage.lane_path()
              .front()
              .lane_id();
      for (const auto &scheduler_output : multi_tasks) {
        if (scheduler_output.drive_passage.lane_path().front().lane_id() ==
            fallback_start_id) {
          AppendFallbackToResultList(output->fallback_status,
                                     std::move(fallback_result), &multi_tasks,
                                     &status_lists, &results);
          break;
        }
      }
    }
  }

  // ---------------------------------------
  // ------ Trajectory Selection ----------------
  // ---------------------------------------
  const auto &vehicle_geom_params =
      input.planner_input->vehicle_params.vehicle_geometry_params();
  const RouteSectionsInfo sections_info(
      *psmm, &async_path_output->route_sections_from_start,
      async_path_output->avoid_lanes);
  const SelectorInput selector_input{
      .smm = psmm->semantic_map_manager(),
      .sections_info = &sections_info,
      .lane_path_infos = &async_path_output->lp_infos,
      .prev_lane_path_from_current =
          &async_path_output->prev_target_lane_path_from_start,
      .prev_traj = input.time_aligned_prev_traj,
      .motion_constraints =
          &input.planner_params->planner_params().motion_constraint_params(),
      .vehicle_geom = &vehicle_geom_params,
      .plan_start_point = &input.start_point_info->start_point,
      .stalled_objects = input.stalled_objects,
      .config = &input.planner_params->planner_params().selector_params()};

  const auto selected_idx_or =
      SelectTrajectory(selector_input, multi_tasks, status_lists, results,
                       &output->selector_debug);
  if (selected_idx_or.ok()) {
    std::swap(multi_tasks[0], multi_tasks[*selected_idx_or]);
    std::swap(results[0], results[*selected_idx_or]);

    if (!multi_tasks[0].is_fallback) {
      std::swap(status_lists[0], status_lists[*selected_idx_or]);
      std::swap(est_debugs[0], est_debugs[*selected_idx_or]);
      std::swap(charts_datas[0], charts_datas[*selected_idx_or]);
    }
  }

  output->scheduler_output_list = std::move(multi_tasks);
  output->est_status_list = std::move(status_lists);
  output->est_planner_output_list = std::move(results);
  output->est_planner_debug_list = std::move(est_debugs);
  output->chart_data_list = std::move(charts_datas);
  output->speed_considered_objects_prediction =
      std::move(speed_considered_objects_prediction);

  return selected_idx_or.ok()
             ? OkPlannerStatus()
             : PlannerStatus(PlannerStatusProto::SELECTOR_FAILED,
                             selected_idx_or.status().message());
}  // namespace planner

}  // namespace planner
}  // namespace qcraft
