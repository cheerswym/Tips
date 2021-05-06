#include "onboard/planner/plan/plan_task_dispatcher.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "onboard/async/async_macro.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/motion_state_filter.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/predicted_motion_filter.h"
#include "onboard/planner/object/reflected_object_in_proximity_filter.h"
#include "onboard/planner/plan/cruise_task.h"
#include "onboard/planner/plan/free_drive_task.h"
#include "onboard/planner/plan/parking_task.h"
#include "onboard/planner/plan/plan_task_helper.h"
#include "onboard/planner/plan/uturn_task.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"

namespace qcraft::planner {

namespace {

GlobalPose ConvertLanePointToGlobalPose(const PlannerSemanticMapManager &psmm,
                                        mapping::LanePoint lane_point,
                                        const CoordinateConverter &cc) {
  const Vec2d global_pos =
      cc.SmoothToGlobal(lane_point.ComputePos(*psmm.semantic_map_manager()));

  return GlobalPose{
      .pos = Vec3d(global_pos.x(), global_pos.y(), 0.0),
      .heading = cc.SmoothYawToGlobalNoNormalize(
          lane_point.ComputeLerpTheta(*psmm.semantic_map_manager()))};
}

absl::StatusOr<mapping::LanePath> AlignLanePathToPose(
    const SemanticMapManager &smm, const mapping::LanePath &lane_path,
    const PoseProto &ego_pose) {
  const auto points = mapping::SampleLanePathPoints(smm, lane_path);
  ASSIGN_OR_RETURN(const auto ff, BuildBruteForceFrenetFrame(points));

  const FrenetCoordinate sl =
      ff.XYToSL(Vec2d(ego_pose.pos_smooth().x(), ego_pose.pos_smooth().y()));

  return lane_path.AfterArclength(sl.s);
}

}  // namespace

absl::Status RunPlanTaskDispatcher(
    const CoordinateConverter &coordinate_converter,
    // TODO(lidong): Remove dependency on the planner params class.
    const PlannerParams &planner_params, const PlannerInput &input,
    const RouteManagerOutput &route_output, const ObjectsProto *objects_proto,
    const TeleopState &teleop_state,
    const GeometryGraphProto::EndInfo &prev_end_info, absl::Time current_time,
    PlannerState *planner_state, PlannerOutput *output, ThreadPool *thread_pool,
    LiteModule *lite_module) {
  const auto &veh_geo_params = input.vehicle_params.vehicle_geometry_params();
  const auto &veh_drive_params = input.vehicle_params.vehicle_drive_params();

  // -------------- Reroute ---------------------
  bool rerouted = route_output.update_id != planner_state->route_update_id;
  planner_state->route_update_id = route_output.update_id;
  if (rerouted) {
    planner_state->plan_task_queue = CreatePlanTasksQueueFromRoutingResult(
        route_output,
        *input.planner_semantic_map_manager->semantic_map_manager());
  }

  auto &plan_task_queue = planner_state->plan_task_queue;

  bool new_task = false;

  // Create new tasks at run time: uturn.
  if (FLAGS_planner_enable_runtime_uturn_task &&
      plan_task_queue.front().type() == ON_ROAD_CRUISE_PLAN) {
    const auto aligned_lane_path_or = AlignLanePathToPose(
        *input.planner_semantic_map_manager->semantic_map_manager(),
        planner_state->prev_target_lane_path, *input.pose);

    if (aligned_lane_path_or.ok()) {
      auto new_tasks_or =
          CreateUturnTask(*input.planner_semantic_map_manager, *input.pose,
                          *aligned_lane_path_or,
                          route_output.route_from_current->lane_path().back());

      if (new_tasks_or.ok()) {
        plan_task_queue.pop_front();
        auto new_tasks = std::move(new_tasks_or).value();
        for (auto it = new_tasks.rbegin(); it != new_tasks.rend(); it++) {
          plan_task_queue.emplace_front(std::move(*it));
        }
        new_task = true;
      }
    }
  }

  if (planner_state->plan_task_queue.size() > 1 &&
      PlanTaskCompeleted(
          planner_state->plan_task_queue.front(), coordinate_converter,
          Vec2d(input.pose->pos_smooth().x(), input.pose->pos_smooth().y()) +
              veh_geo_params.front_edge_to_center() *
                  Vec2d::FastUnitFromAngle(input.pose->yaw()),
          input.pose->yaw(), input.pose->vel_body().x(),
          *input.planner_semantic_map_manager)) {
    // Iterate to next task.
    planner_state->plan_task_queue.pop_front();
    new_task = true;
  }

  new_task = new_task || rerouted;

  // -----------------------------------------------------
  // ------------ Prepare data ---------------------------
  // -----------------------------------------------------

  const bool previously_triggered_aeb =
      input.prev_planner_debug.emergency_stop().has_is_enabled() &&
      input.prev_planner_debug.emergency_stop().is_enabled();
  const auto plan_start_point_info = ComputeEstPlanStartPoint(
      /*predicted_plan_time=*/current_time +
          absl::Milliseconds(FLAGS_planner_lookforward_time_ms),
      planner_state->previous_trajectory, *input.pose, *input.autonomy_state,
      planner_state->previous_autonomy_state, previously_triggered_aeb,
      /*rerouted=*/false,
      /*is_emergency_stop=*/false, *input.chassis,
      planner_params.planner_params().motion_constraint_params(),
      veh_geo_params, veh_drive_params);

  const absl::Time plan_time = plan_start_point_info.plan_time;

  const auto time_aligned_prev_traj_points = CreatePreviousTrajectory(
      plan_time, planner_state->previous_trajectory,
      planner_params.planner_params().motion_constraint_params());

  ObjectVector<PlannerObject> planner_objects =
      FLAGS_planner_consider_objects
          ? BuildPlannerObjects(objects_proto, input.prediction.get(),
                                ToUnixDoubleSeconds(plan_time), thread_pool)
          : ObjectVector<PlannerObject>();
  // Enabled low likelihood object filter and motion state filter.
  const LowLikelihoodFilter low_likelihood_filter(
      FLAGS_planner_prediction_probability_threshold,
      /*only_use_most_likely_traj=*/false);
  const MotionStateFilter motion_state_filter(*input.pose, veh_geo_params);
  const PredictedMotionFilter predicted_motion_filter(planner_objects);
  std::vector<const TrajectoryFilter *> filters = {
      &low_likelihood_filter, &motion_state_filter, &predicted_motion_filter};

  // TODO(lidong): Delete the code after March 01, 2022
  const double filter_reflected_object_distance =
      FLAGS_planner_filter_reflected_object_distance;
  const ReflectedObjectInProximityFilter object_in_proximity_filter(
      *input.pose, veh_geo_params, filter_reflected_object_distance);
  if (filter_reflected_object_distance > 0.0) {
    filters.push_back(&object_in_proximity_filter);
  }

  PlannerObjectManagerBuilder obj_mgr_builder;
  obj_mgr_builder.set_planner_objects(std::move(planner_objects))
      .set_filters(filters);
  FilteredTrajectories pre_filtered;
  auto object_manager_or = obj_mgr_builder.Build(&pre_filtered, thread_pool);
  if (!object_manager_or.ok()) {
    auto *planner_status =
        output->mutable_planner_debug()->mutable_planner_status();
    planner_status->set_status(PlannerStatusProto::OBJECT_MANAGER_FAILED);
    planner_status->set_message(
        std::string(object_manager_or.status().message()));
    return object_manager_or.status();
  }
  auto object_manager = std::move(*object_manager_or);
  auto st_traj_mgr = std::make_unique<SpacetimeTrajectoryManager>(
      absl::Span<const TrajectoryFilter *>{},
      absl::Span<const std::unique_ptr<SpacetimePlannerTrajectoryFinder>>{},
      object_manager.planner_objects(),
      /*st_planner_trajectories_start_offset=*/0.0, thread_pool);

  const absl::Cleanup clean_object_container_async = [&object_manager,
                                                      &st_traj_mgr] {
    MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(object_manager);
    MOVE_DESTROY_CONTAINER_ASYNC_DISPOSAL(st_traj_mgr);
  };

  QCHECK_NOTNULL(input.planner_semantic_map_manager->semantic_map_manager());

  // ---------- Executing current task --------------------
  const auto &current_task = planner_state->plan_task_queue.front();

  switch (current_task.type()) {
    case ON_ROAD_CRUISE_PLAN: {
      //   ----------------- Cruise Task -------------------
      CruiseTaskOutput on_road_result;
      const auto on_road_plan_status = RunCruiseTask(
          CruiseTaskInput{
              .rerouted = rerouted,
              .coordinate_converter = &coordinate_converter,
              .planner_input = &input,
              .planner_params = &planner_params,
              .teleop_state = &teleop_state,
              .route_output = &route_output,
              .plan_start_point_info = &plan_start_point_info,
              .plan_time = plan_time,
              .st_traj_mgr = st_traj_mgr.get(),
              .object_manager = &object_manager,
              .time_aligned_prev_traj_points = &time_aligned_prev_traj_points,
              .prev_end_info = &prev_end_info},
          &on_road_result, planner_state, thread_pool, lite_module);

      *output->mutable_trajectory() = std::move(on_road_result.trajectory_info);
      *output->mutable_planner_debug() = std::move(on_road_result.debug_info);
      *output->mutable_charts_data() = std::move(on_road_result.chart_data);
      *output->mutable_hmi_content() = std::move(on_road_result.hmi_content);

      if (!on_road_plan_status.ok()) {
        return absl::NotFoundError(on_road_plan_status.message());
      }

      break;
    }
    case UTURN_PLAN: {
      //   ----------------- Uturn Task -------------------
      GlobalPose goal;
      if (current_task.destination_info().dest.lane_points.has_value()) {
        goal = ConvertLanePointToGlobalPose(
            *input.planner_semantic_map_manager,
            current_task.destination_info().dest.lane_points->front(),
            coordinate_converter);
      } else {
        if (!current_task.destination_info().dest.global_pose.has_value()) {
          return absl::UnavailableError("Task destination must be specified.");
        }
        goal = *current_task.destination_info().dest.global_pose;
      }

      mapping::LanePath uturn_ref_lane_path(
          input.planner_semantic_map_manager->semantic_map_manager());
      if (current_task.destination_info().uturn_ref_lane_path.has_value()) {
        uturn_ref_lane_path.FromProto(
            current_task.destination_info().uturn_ref_lane_path.value());
      }

      UTurnTaskOutput result;
      const auto status = RunUTurnTask(
          UTurnTaskInput{
              .reset = new_task,
              .psmm = input.planner_semantic_map_manager.get(),
              .coordinate_converter = &coordinate_converter,
              .goal = &goal,
              .lane_path = uturn_ref_lane_path.IsEmpty() ? nullptr
                                                         : &uturn_ref_lane_path,
              .pose = input.pose.get(),
              .chassis = input.chassis.get(),
              .plan_start_point_info = &plan_start_point_info,
              .plan_time = plan_time,
              .planner_params = &planner_params,
              .veh_geo_params = &veh_geo_params,
              .veh_drive_params = &veh_drive_params,
              .prev_trajectory_proto = &planner_state->previous_trajectory,
              .object_manager = &object_manager},
          &planner_state->freespace_planner_state, &result, thread_pool);

      *output->mutable_trajectory() = std::move(result.trajectory_info);
      *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
          std::move(result.debug_proto);
      *output->mutable_charts_data() = std::move(result.chart_data);

      if (!status.ok()) {
        return absl::UnavailableError(status.ToString());
      }

      break;
    }
    case OFF_ROAD_PLAN: {
      if (current_task.destination_info().dest.parking_spots.has_value()) {
        //   ----------------- Parking Task -------------------
        ParkingTaskOutput result;
        const auto status = RunParkingTask(
            ParkingTaskInput{
                .reset = new_task,
                .psmm = input.planner_semantic_map_manager.get(),
                .coordinate_converter = &coordinate_converter,
                .parking_spot_id =
                    current_task.destination_info().dest.parking_spots->front(),
                .pose = input.pose.get(),
                .chassis = input.chassis.get(),
                .plan_start_point_info = &plan_start_point_info,
                .plan_time = plan_time,
                .planner_params = &planner_params,
                .veh_geo_params = &veh_geo_params,
                .veh_drive_params = &veh_drive_params,
                .prev_trajectory_proto = &planner_state->previous_trajectory,
                .object_manager = &object_manager},
            &planner_state->freespace_planner_state, &result, thread_pool);

        *output->mutable_trajectory() = std::move(result.trajectory_info);
        *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
            std::move(result.debug_proto);
        *output->mutable_charts_data() = std::move(result.chart_data);

        if (!status.ok()) {
          return absl::UnavailableError(status.ToString());
        }

      } else {
        //   ----------------- Free Drive Task -------------------
        GlobalPose goal;
        if (current_task.destination_info().dest.lane_points.has_value()) {
          goal = ConvertLanePointToGlobalPose(
              *input.planner_semantic_map_manager,
              current_task.destination_info().dest.lane_points->front(),
              coordinate_converter);
        } else {
          if (!current_task.destination_info().dest.global_pose.has_value()) {
            return absl::UnavailableError(
                "Task destination must be specified.");
          }
          goal = *current_task.destination_info().dest.global_pose;
        }
        FreeDriveTaskOutput result;
        const auto status = RunFreeDriveTask(
            FreeDriveTaskInput{
                .reset = new_task,
                .psmm = input.planner_semantic_map_manager.get(),
                .coordinate_converter = &coordinate_converter,
                .goal = &goal,
                .pose = input.pose.get(),
                .chassis = input.chassis.get(),
                .plan_start_point_info = &plan_start_point_info,
                .plan_time = plan_time,
                .planner_params = &planner_params,
                .veh_geo_params = &veh_geo_params,
                .veh_drive_params = &veh_drive_params,
                .prev_trajectory_proto = &planner_state->previous_trajectory,
                .object_manager = &object_manager},
            &planner_state->freespace_planner_state, &result, thread_pool);

        *output->mutable_trajectory() = std::move(result.trajectory_info);
        *output->mutable_planner_debug()->mutable_freespace_planner_debug() =
            std::move(result.debug_proto);
        *output->mutable_charts_data() = std::move(result.chart_data);

        if (!status.ok()) {
          return absl::UnavailableError(status.ToString());
        }
      }
      break;
    }
    case BLOCKED_PLAN:
      return absl::UnavailableError("Not supported yet.");
  }

  planner_state->previous_trajectory = output->trajectory();
  if (input.chassis->parking_brake()) {
    planner_state->parking_brake_release_time = plan_time;
  }

  planner_state->previous_autonomy_state = *input.autonomy_state;
  ConvertTrajectoryToGlobalCoordinates(
      coordinate_converter, output->trajectory(),
      &planner_state->previous_trajectory_global,
      &planner_state->previous_past_trajectory_global);

  return absl::OkStatus();
}

}  // namespace qcraft::planner
