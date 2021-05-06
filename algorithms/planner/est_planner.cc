#include "onboard/planner/est_planner.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <regex>
#include <utility>

#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/auto_tuning_common_flags.h"
#include "offboard/vis/vantage/charts/chart_util.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/async_macro.h"
#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/initializer_util.h"
#include "onboard/planner/initializer/search_motion.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_input.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_output.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/plan/st_path_planner.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/speed/speed_finder_output.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/trajectory_validation.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/trajectory_plot_util.h"
#include "onboard/vis/common/colormap.h"

namespace qcraft {
namespace planner {

PlannerStatus RunEstPlanner(const EstPlannerInput &input,
                            EstPlannerOutput *est_output,
                            ThreadPool *thread_pool) {
  SCOPED_QTRACE("EstPlanner");
  const auto &vehicle_params = *input.vehicle_params;
  const auto &vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto &vehicle_drive_params = vehicle_params.vehicle_drive_params();
  const VehicleModel vehicle_model = vehicle_params.vehicle_params().model();

  const auto path_look_ahead_time =
      GetStPathPlanLookAheadTime(*input.start_point_info, *input.pose,
                                 input.planner_state->previous_trajectory);
  const auto path_start_point_info =
      GetStPathPlanStartPointInfo(path_look_ahead_time, *input.start_point_info,
                                  input.planner_state->previous_trajectory);

  const StPathPlannerInput st_path_input{
      .plan_id = input.plan_id,
      .start_point_info = &path_start_point_info,
      .scheduler_output = input.scheduler_output,
      .vehicle_geom_params = &vehicle_geom_params,
      .vehicle_drive_params = &vehicle_drive_params,
      .planner_params = input.planner_params,
      .obj_mgr = input.obj_mgr,
      .planner_semantic_map_manager = input.planner_semantic_map_manager,
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .tl_info_map = input.tl_info_map,
      .pose = input.pose,
      .teleop_state = input.teleop_state,
      .vehicle_model = &vehicle_model,
      .time_aligned_prev_traj = input.time_aligned_prev_traj,
      .sensor_fovs = input.sensor_fovs,
      .prev_st_planner_trajectories =
          &input.planner_state->st_planner_trajectories,
      .prev_decider_state = &input.planner_state->decider_state,
      .prev_initializer_state = &input.planner_state->initializer_state,
      .parking_brake_release_time =
          input.planner_state->parking_brake_release_time,
  };
  StPathPlannerOutput path_output;
  if (auto path_status =
          RunStPathPlanner(st_path_input, &path_output, thread_pool);
      !path_status.ok()) {
    return path_status;
  }

  const auto &scheduler_output = *input.scheduler_output;
  const auto &drive_passage = scheduler_output.drive_passage;
  const auto &sl_boundary = scheduler_output.sl_boundary;

  // Run speed.
  SpeedFinderInput speed_input;
  speed_input.base_name = "est";
  speed_input.traj_mgr = path_output.traj_mgr.get();
  speed_input.psmm = input.planner_semantic_map_manager;
  speed_input.constraint_mgr = &path_output.constraint_manager;
  speed_input.drive_passage = &drive_passage;
  speed_input.route_sections_from_start = input.route_sections_from_start;
  speed_input.path_sl_boundary = &sl_boundary;
  speed_input.stalled_objects = input.stalled_objects;
  speed_input.path = &path_output.path;
  // speed planning always starts at plan start point
  speed_input.plan_start_point = input.start_point_info->start_point;

  const auto &motion_constraint_params =
      input.planner_params->motion_constraint_params();
  const auto &speed_finder_params = input.planner_params->speed_finder_params();

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
      *input.semantic_map_manager, speed_output.considered_st_objects,
      input.start_point_info->full_stop, *input.pose, scheduler_output,
      vehicle_geom_params, vehicle_drive_params, motion_constraint_params,
      est_output->traj_points, &est_output->debug_info.traj_validation_result,
      thread_pool);
  if (!valid && FLAGS_est_fallback_upon_traj_validation_failure) {
    return PlannerStatus(
        PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
        absl::StrCat(
            "Validation failed: ",
            est_output->debug_info.traj_validation_result.DebugString()));
  }

  est_output->chart_data.MergeFrom(std::move(path_output.chart_data));
  // Set cross-frame state of the decider.
  est_output->decider_state = std::move(path_output.decider_state);
  // Set cross-frame state of the initializer.
  est_output->initializer_state = std::move(path_output.initializer_state);
  est_output->considered_st_objects =
      std::move(speed_output.considered_st_objects);
  est_output->trajectory_end_info = std::move(speed_output.trajectory_end_info);
  est_output->feature_costs = std::move(path_output.opt_feature_costs);
  est_output->expert_costs = std::move(path_output.opt_expert_costs);
  est_output->debug_info.optimizer_debug_proto =
      std::move(path_output.optimizer_debug_proto);
  est_output->debug_info.initializer_debug_proto =
      std::move(path_output.initializer_debug_proto);

  // Set cross-frame state of spacetime planner objects.
  for (const auto &st_planner_traj_info :
       path_output.traj_mgr->spacetime_planner_trajs_info()) {
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
      ->Reserve(path_output.traj_mgr->ignored_trajectories().size());
  for (const auto &ignored : path_output.traj_mgr->ignored_trajectories()) {
    auto *filtered =
        est_output->debug_info.filtered_prediction_trajectories.add_filtered();
    filtered->set_reason(ignored.reason);
    filtered->set_id(ignored.object_id);
    filtered->set_index(ignored.traj->index());
  }
  est_output->filtered_traj_mgr = std::move(path_output.traj_mgr);

  MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(path_output);

  return OkPlannerStatus();
}

}  // namespace planner
}  // namespace qcraft
