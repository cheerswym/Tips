#include "onboard/planner/plan/st_path_planner.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_model/vehicle_model.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/constraint_builder.h"
#include "onboard/planner/decision/decider_input.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/search_motion.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager_builder.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_input.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

namespace {

// TODO(jingqiao): Use input instead of directly reading file.
void UpdateTrajOptParams(const std::string &traj_opt_params_address,
                         PlannerParamsProto *mutable_planner_params) {
  auto *traj_opt_params =
      mutable_planner_params->mutable_trajectory_optimizer_params();
  VLOG(3) << "Trajectory optimizer params before update: ";
  VLOG(3) << traj_opt_params->DebugString();
  if (!file_util::TextFileToProto(traj_opt_params_address, traj_opt_params)) {
    QCHECK(false) << "Read trajectory optimizer params as text file failed!!!!";
  }
  QLOG(INFO) << "New trajectory optimizer params are used.";
}

std::vector<ApolloTrajectoryPointProto> StitchPreviousTrajectoryAndStTrajectory(
    int stitch_index,
    const std::vector<ApolloTrajectoryPointProto> &time_aligned_prev_traj,
    const std::vector<ApolloTrajectoryPointProto> &st_trajectory) {
  if (time_aligned_prev_traj.empty() || stitch_index <= 0) {
    return st_trajectory;
  }
  std::vector<ApolloTrajectoryPointProto> res_traj;
  res_traj.reserve(stitch_index + st_trajectory.size());
  for (int idx = 0; idx < stitch_index; ++idx) {
    res_traj.push_back(time_aligned_prev_traj[idx]);
  }
  const double s_offset = time_aligned_prev_traj[stitch_index].path_point().s();
  for (const auto &point : st_trajectory) {
    res_traj.push_back(point);
    res_traj.back().mutable_path_point()->set_s(point.path_point().s() +
                                                s_offset);
  }
  return res_traj;
}

}  // namespace

PlannerStatus RunStPathPlanner(const StPathPlannerInput &input,
                               StPathPlannerOutput *out,
                               ThreadPool *thread_pool) {
  SCOPED_QTRACE("SpacetimePathPlanner");

  const Vec2d plan_start_pos(
      ToVec2d(input.start_point_info->start_point.path_point()));
  const auto *drive_passage = &input.scheduler_output->drive_passage;
  const auto *sl_boundary = &input.scheduler_output->sl_boundary;

  // Build drive passage.
  // TODO(xiangjun): temporary hack for testing scheduler path sl boundary.
  if (const auto _ = drive_passage->QueryFrenetCoordinateAt(plan_start_pos);
      !_.ok()) {
    return PlannerStatus(PlannerStatusProto::INPUT_INCORRECT,
                         absl::StrCat("Failed to project plan start pos.",
                                      _.status().ToString()));
  }

  SpacetimeTrajectoryManagerBuilderInput st_mgr_builder_input{
      .passage = drive_passage,
      .sl_boundary = sl_boundary,
      .obj_mgr = input.obj_mgr,
      .veh_geom = input.vehicle_geom_params,
      .plan_start_point = &input.start_point_info->start_point,
      .st_planner_trajectories_start_index =
          input.start_point_info->relative_index_from_plan_start_point,
      .lane_change_state = &input.scheduler_output->lane_change_state,
      .prev_st_trajs = input.prev_st_planner_trajectories};
  out->traj_mgr =
      BuildSpacetimeTrajectoryManager(st_mgr_builder_input, thread_pool);

  if (FLAGS_planner_drive_passage_debug_level) {
    SendDrivePassageToCanvas(*drive_passage,
                             absl::StrCat("drive_passage_", input.plan_id));
  }

  // Build constraint manager.
  // TODO(yumeng): Serialize this decision context if it need to be persist
  // across iterations.
  DeciderInput decider_input{
      .vehicle_geometry_params = input.vehicle_geom_params,
      .config = &input.planner_params->decision_constraint_config(),
      .planner_semantic_map_manager = input.planner_semantic_map_manager,
      .stalled_objects = input.stalled_objects,
      .scene_reasoning = input.scene_reasoning,
      .lc_stage = input.scheduler_output->lane_change_state.stage(),
      .plan_start_point = &input.start_point_info->start_point,
      .lane_path_before_lc = &input.scheduler_output->lane_path_before_lc,
      .passage = drive_passage,
      .sl_boundary = sl_boundary,
      .borrow_lane_boundary = input.scheduler_output->borrow_lane,
      .obj_mgr = input.obj_mgr,
      .st_traj_mgr = out->traj_mgr.get(),
      .tl_info_map = input.tl_info_map,
      .lc_clearance_check_output = &input.scheduler_output->clearance_output,
      .pre_decider_state = input.prev_decider_state,
      .pose = input.pose,
      .av_frenet_box = &input.scheduler_output->av_frenet_box_on_drive_passage,
      .parking_brake_release_time = input.parking_brake_release_time,
      .teleop_enable_traffic_light_stop =
          input.teleop_state->enable_traffic_light_stopping,
      .enable_pull_over = input.teleop_state->enable_pull_over,
      .brake_to_stop = input.teleop_state->brake_to_stop,
      .length_along_route = input.scheduler_output->length_along_route,
      .vehicle_model = *input.vehicle_model,
      .plan_time = input.start_point_info->plan_time,
      .sensor_fovs = input.sensor_fovs};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto decider_output, BuildConstraints(decider_input),
      PlannerStatusProto::DECISION_CONSTRAINTS_UNAVAILABLE);

  // Update spacetime planner trajectory from leading obj.
  for (const auto &[traj_id, leading_object] :
       decider_output.constraint_manager.LeadingObjects()) {
    const auto status = out->traj_mgr->AddSpaceTimePlannerTrajectoryById(
        traj_id, SpacetimePlannerTrajectoryReason::LEADING);
    VLOG_IF(3, !status.ok()) << status.ToString();
  }

  // Run initializer.
  InitializerInput initializer_input{
      .start_point_info = input.start_point_info,
      .sdc_pose = input.pose,
      .lane_change_state = &input.scheduler_output->lane_change_state,
      .lc_multiple_traj = decider_output.initializer_lc_multiple_traj,
      .lc_clearance = &input.scheduler_output->clearance_output,
      .stalled_objects = input.stalled_objects,
      .drive_passage = drive_passage,
      .st_traj_mgr = out->traj_mgr.get(),
      .vehicle_geom = input.vehicle_geom_params,
      .vehicle_drive = input.vehicle_drive_params,
      .constraint_manager = &decider_output.constraint_manager,
      .sl_boundary = sl_boundary,
      .prev_initializer_state = input.prev_initializer_state,
      .planner_params = input.planner_params,
      .plan_id = input.plan_id,
  };

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto initializer_output,
      RunInitializer(initializer_input, &out->initializer_debug_proto,
                     &out->chart_data, thread_pool),
      PlannerStatusProto::INITIALIZER_FAILED);

  // Add lane change target decision to constraint manager.
  if (initializer_output.lc_targets.has_value()) {
    for (const auto &target : *initializer_output.lc_targets) {
      decider_output.constraint_manager.AddLeadingObject(target);
    }
  }

  // Update constraint manager.
  if (initializer_output.blocking_static_obj != nullptr) {
    if (!decider_output.constraint_manager.IsLeadingObject(
            initializer_output.blocking_static_obj->traj_id())) {
      // Stalled blocking static is not a leading object.
      const auto *traj = QCHECK_NOTNULL(out->traj_mgr->FindTrajectoryById(
          initializer_output.blocking_static_obj->traj_id()));
      if (!input.stalled_objects->contains(std::string(traj->object_id()))) {
        decider_output.constraint_manager.AddLeadingObject(
            *initializer_output.blocking_static_obj);
      }
    }
    // Update spacetime planner trajectory. It might not be a leading
    // object but should be considered since potential collision.
    const auto status = out->traj_mgr->AddSpaceTimePlannerTrajectoryById(
        initializer_output.blocking_static_obj->traj_id(),
        SpacetimePlannerTrajectoryReason::LEADING);
    VLOG_IF(3, !status.ok()) << status.ToString();
  }

  out->initializer_state = std::move(initializer_output.initializer_state);
  auto mutable_planner_params = *input.planner_params;

  // TODO(Runbing): Change to shift by index directly.
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory =
      ShiftTrajectoryByTime(
          input.start_point_info->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          *input.time_aligned_prev_traj,
          mutable_planner_params.motion_constraint_params().max_accel_jerk(),
          mutable_planner_params.motion_constraint_params().max_decel_jerk());
  // Run optimizer.
  TrajectoryOptimizerInput opt_input{
      .trajectory = initializer_output.search_result.traj_points,
      .previous_trajectory = previous_trajectory,
      .st_traj_mgr = out->traj_mgr.get(),
      .drive_passage = drive_passage,
      .path_sl_boundary = sl_boundary,
      .constraint_mgr = &decider_output.constraint_manager,
      .planner_semantic_map_mgr = input.planner_semantic_map_manager,
      .plan_start_point = input.start_point_info->start_point,
      .plan_start_time = input.start_point_info->plan_time,
      .plan_id = input.plan_id};
  // BANDAID(jingqiao): Refactor to solve code divergence in the future.
  if (FLAGS_auto_tuning_mode || FLAGS_compare_different_weight) {
    UpdateTrajOptParams(FLAGS_traj_opt_params_file_address,
                        &mutable_planner_params);
    // For readability.
  } else if (FLAGS_update_learned_alphas) {
    if (FLAGS_update_learned_alphas_except_lane_change) {
      if (input.scheduler_output->lane_change_state.stage() ==
          LaneChangeStage::LCS_NONE) {
        UpdateTrajOptParams(FLAGS_traj_opt_params_file_address,
                            &mutable_planner_params);
      }
    } else {
      UpdateTrajOptParams(FLAGS_traj_opt_params_file_address,
                          &mutable_planner_params);
    }
  }
  VLOG(3) << "Actual cost weights used in trajectory optimizer: ";
  VLOG(3) << mutable_planner_params.trajectory_optimizer_params()
                 .cost_weight_params()
                 .DebugString();

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto opt_output,
      OptimizeTrajectory(
          opt_input, mutable_planner_params, *input.vehicle_geom_params,
          *input.vehicle_drive_params, &out->optimizer_debug_proto,
          &out->chart_data, thread_pool),
      PlannerStatusProto::OPTIMIZER_FAILED);

  out->opt_feature_costs = std::move(opt_output.feature_costs);
  out->opt_expert_costs = std::move(opt_output.expert_costs);
  out->decider_state = std::move(decider_output.decider_state);

  if (FLAGS_compare_different_weight) {
    TrajectoryOptimizerDebugProto original_optimizer_debug;
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        const auto original_opt_output,
        OptimizeTrajectory(
            opt_input, *input.planner_params, *input.vehicle_geom_params,
            *input.vehicle_drive_params, &original_optimizer_debug,
            &out->chart_data, thread_pool),
        PlannerStatusProto::PLANNER_ABNORMAL_EXIT);
    const std::string base_name =
        absl::StrFormat("traj_opt_%d", opt_input.plan_id);
    optimizer::AddCompareTrajCanvas(base_name, original_opt_output.trajectory,
                                    "original_weight", opt_output.trajectory,
                                    "auto_tuned_weight");
    if (FLAGS_compare_based_on_original_weight) {
      opt_output = std::move(original_opt_output);
      out->optimizer_debug_proto = std::move(original_optimizer_debug);
    }
  }

  out->constraint_manager = std::move(decider_output.constraint_manager);

  // Path Extension
  QCHECK(!opt_output.trajectory_proto.empty());

  std::vector<ApolloTrajectoryPointProto> st_trajectory =
      StitchPreviousTrajectoryAndStTrajectory(
          input.start_point_info->relative_index_from_plan_start_point,
          *input.time_aligned_prev_traj, opt_output.trajectory_proto);

  constexpr double kRequiredMinPathLength = 15.0;
  const double max_curvature = GetCenterMaxCurvature(
      *input.vehicle_geom_params, *input.vehicle_drive_params);
  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto extended_path,
      ExtendPathAndDeleteUnreasonablePart(
          st_trajectory, &input.scheduler_output->drive_passage,
          &input.scheduler_output->sl_boundary, kRequiredMinPathLength,
          max_curvature),
      PlannerStatusProto::PATH_EXTENSION_FAILED);
  out->path = std::move(extended_path);

  return OkPlannerStatus();
}

}  // namespace qcraft::planner
