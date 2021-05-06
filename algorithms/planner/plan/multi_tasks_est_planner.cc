#include "onboard/planner/plan/multi_tasks_est_planner.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "onboard/async/parallel_for.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/est_planner.h"
#include "onboard/planner/plan/expert_planner.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/fallback_planner_debug.pb.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/multi_tasks_scheduler.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/scheduler/target_lane_path_filter.h"
#include "onboard/planner/selector/selector.h"
#include "onboard/planner/util/planner_status_macros.h"

namespace qcraft::planner {

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

void AppendExpertToResultList(const PlannerStatus &expert_status,
                              ExpertPlannerOutput expert_result,
                              std::vector<SchedulerOutput> *multi_tasks,
                              std::vector<PlannerStatus> *status_list,
                              std::vector<EstPlannerOutput> *results) {
  status_list->push_back(expert_status);
  multi_tasks->push_back(std::move(expert_result.scheduler_output));
  results->emplace_back(EstPlannerOutput{
      .traj_points = std::move(expert_result.trajectory_points),
      .decider_state = std::move(expert_result.decider_state),
      .filtered_traj_mgr = std::move(expert_result.filtered_traj_mgr),
      .considered_st_objects = std::move(expert_result.considered_st_objects),
      .trajectory_end_info = std::move(expert_result.trajectory_end_info)});
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

}  // namespace

PlannerStatus RunMultiTasksEstPlanner(const MultiTasksEstPlannerInput &input,
                                      PathBoundedEstPlannerOutput *output,
                                      ThreadPool *thread_pool) {
  SCOPED_QTRACE("MultiTasksEstPlan");

  const auto &plan_start_point = input.start_point_info->start_point;
  const auto &vehicle_geometry =
      input.planner_input->vehicle_params.vehicle_geometry_params();
  const auto &smm = *input.planner_input->planner_semantic_map_manager
                         ->semantic_map_manager();
  const auto &psmm = *input.planner_input->planner_semantic_map_manager;

  ScopedMultiTimer timer("multi_tasks_est");

  // ----------------------------------------------------------
  // ---------------- Project plan_start_point ----------------
  // ----------------------------------------------------------
  const RouteSectionsInfo route_sections_info_from_start(
      psmm, input.route_sections_from_start, input.route->avoid_lanes());
  // Guarantee that the lane path starts from the current section.
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      const auto prev_target_lane_path_from_start,
      AlignLanePathWithRouteSections(
          route_sections_info_from_start, *input.prev_target_lane_path,
          kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength),
      PlannerStatusProto::START_POINT_PROJECTION_TO_ROUTE_FAILED);
  timer.Mark("project_start_point");

  // ------------------------------------------------------------------
  // --------------------- Fallback planner ---------------------------
  // ------------------------------------------------------------------
  FallbackPlannerOutput fallback_result;
  Future<PlannerStatus> future_fallback_status =
      ScheduleFuture(thread_pool, [&]() {
        FallbackPlannerInput fallback_input{
            .psmm = input.planner_input->planner_semantic_map_manager.get(),
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

  // --------------------------------------------------------------
  // ----------------- Choose candidates on lane graph ------------
  // --------------------------------------------------------------
  const auto lane_graph = BuildLaneGraph(
      psmm, route_sections_info_from_start, *input.object_manager,
      *input.stalled_objects, input.route->avoid_lanes());
  if (FLAGS_planner_send_lane_graph_to_canvas) {
    SendLaneGraphToCanvas(lane_graph, smm, route_sections_info_from_start,
                          "planner/lane_graph");
  }

  // Collect candidate lane paths from each start lane. If one or more
  // diverging point exists starting from some lane, the first one would be
  // considered, thus two candidates would be produced.
  auto lp_infos = FindBestLanePathsFromStart(
      psmm, route_sections_info_from_start, lane_graph, thread_pool);
  if (lp_infos.empty()) {
    // In case all lane paths are blocked by stalled objects.
    return PlannerStatus(PlannerStatusProto::PLANNER_ABNORMAL_EXIT,
                         "No viable candidate route found to destination.");
  }

  // ------------------------------------------------------------------
  // --------------------- Optional expert planner --------------------
  // ------------------------------------------------------------------
  ExpertPlannerOutput expert_result;
  Future<PlannerStatus> future_expert_status;
  if (FLAGS_dumping_selector_features) {
    future_expert_status = ScheduleFuture(thread_pool, [&]() {
      ExpertPlannerInput expert_planner_input{
          .psmm = input.planner_input->planner_semantic_map_manager.get(),
          .pose = input.planner_input->pose.get(),
          .start_point_info = input.start_point_info,
          .prev_target_lane_path_from_start = &prev_target_lane_path_from_start,
          .station_anchor = input.station_anchor,
          .sections_info_from_start = &route_sections_info_from_start,
          .obj_mgr = input.object_manager,
          .st_traj_mgr = input.st_traj_mgr,
          .stalled_objects = input.stalled_objects,
          .scene_reasoning = input.scene_reasoning,
          .traffic_light_states =
              input.planner_input->traffic_light_states.get(),
          .pre_decider_state = &input.planner_state->decider_state,
          .tl_info_map = input.tl_info_map,
          .parking_brake_release_time =
              input.planner_state->parking_brake_release_time,
          .teleop_enable_traffic_light_stop =
              input.teleop_state->enable_traffic_light_stopping,
          .enable_pull_over = input.teleop_state->enable_pull_over,
          .brake_to_stop = input.teleop_state->brake_to_stop,
          .lp_infos = &lp_infos,
          .planner_params = &input.planner_params->planner_params(),
          .vehicle_params = &input.planner_input->vehicle_params,
          .smooth_result_map = input.smooth_result_map,
          .prev_smooth_state = input.planner_state->prev_smooth_state,
          .sensor_fovs = input.planner_input->sensor_fovs.get(),
      };

      return RunExpertPlanner(expert_planner_input, &expert_result,
                              &output->expert_chart_data, &output->expert_debug,
                              thread_pool);
    });
  }

  const absl::Cleanup wait_for_expert = [&future_expert_status, &output]() {
    if (future_expert_status.IsValid()) {
      output->expert_status = future_expert_status.Get();
    }
  };

  const auto target_lp_infos = FilterMultipleTargetLanePath(
      route_sections_info_from_start, prev_target_lane_path_from_start,
      plan_start_point, input.planner_state->preferred_lane_path, &lp_infos);
  timer.Mark("filter_target_lane_paths");

  // --------------------------------------------------------------
  // ----------------- Run Scheduler ------------------------------
  // --------------------------------------------------------------
  MultiTasksSchedulerInput scheduler_input{
      .psmm = &psmm,
      .vehicle_geom = &vehicle_geometry,
      .planner_params = &input.planner_params->planner_params(),
      .st_traj_mgr = input.st_traj_mgr,
      .stalled_objects = input.stalled_objects,
      .lane_path_infos = &lp_infos,
      .sections_info_from_current = &route_sections_info_from_start,
      .tl_info_map = input.tl_info_map,
      .prev_smooth_state = input.prev_smooth_state,
      .plan_start_point = &plan_start_point,
      .station_anchor = input.station_anchor,
      .start_route_s = 0.0,
      .prev_lane_path_from_current = &prev_target_lane_path_from_start,
      .smooth_result_map = input.smooth_result_map,
      .thread_pool = thread_pool};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto multi_tasks,
      ScheduleMultiplePlanTasks(scheduler_input, target_lp_infos, thread_pool),
      PlannerStatusProto::SCHEDULER_UNAVAILABLE);
  timer.Mark("scheduler");

  // ---------------------------------------
  // ------ Run Est Planner ----------------
  // ---------------------------------------
  std::vector<PlannerStatus> status_lists(multi_tasks.size());
  std::vector<EstPlannerOutput> results(multi_tasks.size());
  std::vector<EstPlannerDebug> est_debugs(multi_tasks.size());
  std::vector<vis::vantage::ChartDataBundleProto> charts_datas(
      multi_tasks.size());

  ParallelFor(0, multi_tasks.size(), thread_pool, [&](int i) {
    status_lists[i] = RunEstPlanner(
        EstPlannerInput{
            .semantic_map_manager = &smm,
            .planner_semantic_map_manager = &psmm,
            .plan_id = i + 1,
            .pose = input.planner_input->pose.get(),
            .planner_state = input.planner_state,
            .vehicle_params = &input.planner_input->vehicle_params,
            .planner_params = &input.planner_params->planner_params(),
            .obj_mgr = input.object_manager,
            .start_point_info = input.start_point_info,
            .tl_info_map = input.tl_info_map,
            .stalled_objects = input.stalled_objects,
            .scene_reasoning = input.scene_reasoning,
            .scheduler_output = &multi_tasks[i],
            .route_sections_from_start = input.route_sections_from_start,
            .time_aligned_prev_traj = input.time_aligned_prev_traj,
            .teleop_state = input.teleop_state,
            .sensor_fovs = input.planner_input->sensor_fovs.get(),
        },
        &results[i], thread_pool);
    est_debugs[i] = std::move(results[i].debug_info);
    charts_datas[i] = std::move(results[i].chart_data);
  });

  // Collect speed-considered object ids by all est planners and fallback
  // planner.
  absl::flat_hash_set<std::string> speed_considered_object_ids;
  for (int i = 0; i < multi_tasks.size(); ++i) {
    for (const auto &part_st_traj : results[i].considered_st_objects) {
      speed_considered_object_ids.insert(
          std::string(part_st_traj.st_traj().object_id()));
    }
    if (!status_lists[i].ok()) {
      QLOG(WARNING) << "Failed task " << i << ": " << status_lists[i].message();
    }
  }
  timer.Mark("multi_trajectories");

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

  if (FLAGS_dumping_selector_features) {
    output->expert_status = future_expert_status.Get();
    if (!output->expert_status.ok()) {
      return output->expert_status;
    }
    AppendExpertToResultList(output->expert_status, std::move(expert_result),
                             &multi_tasks, &status_lists, &results);
  }

  // ---------------------------------------
  // ------ Trajectory Selection ----------------
  // ---------------------------------------
  SelectorInput selector_input{
      .smm = &smm,
      .sections_info = &route_sections_info_from_start,
      .lane_path_infos = &lp_infos,
      .prev_lane_path_from_current = &prev_target_lane_path_from_start,
      .prev_traj = input.time_aligned_prev_traj,
      .motion_constraints =
          &input.planner_params->planner_params().motion_constraint_params(),
      .vehicle_geom = &vehicle_geometry,
      .plan_start_point = &plan_start_point,
      .stalled_objects = input.stalled_objects,
      .prediction_debug = &input.planner_input->prediction_debug,
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
}

}  // namespace qcraft::planner
