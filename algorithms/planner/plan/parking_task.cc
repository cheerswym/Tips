#include "onboard/planner/plan/parking_task.h"

#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/driving_state.h"
#include "onboard/planner/emergency_stop.h"
#include "onboard/planner/freespace/freespace_constraint_builder.h"
#include "onboard/planner/freespace/freespace_planner.h"
#include "onboard/planner/freespace/path_manager.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/scene/off_road_scene_reasoning.h"

namespace qcraft::planner {

namespace {

absl::Status ParkingTaskMain(const ParkingTaskInput &input,
                             FreespacePlannerStateProto *state,
                             ParkingTaskOutput *result,
                             ThreadPool *thread_pool) {
  const auto *parking_spot_info =
      input.psmm->semantic_map_manager()->FindParkingSpotByIdOrNull(
          input.parking_spot_id);

  if (parking_spot_info == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Parking spot ", input.parking_spot_id, " not found."));
  }

  FreespaceTaskProto::TaskType task_type;
  switch (parking_spot_info->type()) {
    case mapping::ParkingSpotInfo::Type::PERPENDICULAR:
      task_type = FreespaceTaskProto::PERPENDICULAR_PARKING;
      break;
    case mapping::ParkingSpotInfo::Type::PARALLEL:
      task_type = FreespaceTaskProto::PARALLEL_PARKING;
      break;
  }

  result->debug_proto.mutable_freespace_task_debug()->set_parking_spot_id(
      input.parking_spot_id);
  result->debug_proto.mutable_freespace_task_debug()->set_task_type(task_type);
  // Scene reasoning in unstructured road. Currently output stalled objects
  // only.
  absl::flat_hash_set<std::string> stalled_objects;
  ASSIGN_OR_RETURN(
      const auto scene_reasoning_output,
      RunOffRoadSceneReasoning(OffRoadSceneReasoningInput{
          .psmm = input.psmm, .object_mgr = input.object_manager}));
  for (const auto &object_annotation :
       scene_reasoning_output.objects_annotation()) {
    // Check stalled object probability, default threshold value is 0.6.
    constexpr double kStalledProbThreshold = 0.6;
    if (object_annotation.stalled_vehicle_likelyhood() >
        kStalledProbThreshold) {
      stalled_objects.insert(object_annotation.object_id());
      result->debug_proto.add_stalled_object_ids(object_annotation.object_id());
    }
  }

  // Reset goal
  if (input.reset || !state->path_manager_state().has_goal()) {
    ASSIGN_OR_RETURN(
        const auto path_point_goal,
        GenerateGoalFromParkingSpot(*input.veh_geo_params, *parking_spot_info,
                                    *input.object_manager));
    const auto global_pos = input.coordinate_converter->SmoothToGlobal(
        Vec2d(path_point_goal.x(), path_point_goal.y()));
    PathManagerStateProto::GlobalGoal global_goal;
    global_pos.ToProto(global_goal.mutable_pos());
    global_goal.set_theta(
        input.coordinate_converter->SmoothYawToGlobalNoNormalize(
            path_point_goal.theta()));
    state->mutable_path_manager_state()->CopyFrom(
        PathManager::MakeInitState(global_goal, path_point_goal, task_type));
  }

  // Freespace map
  ASSIGN_OR_RETURN(
      const auto freespace_map,
      ConstructFreespaceMap(task_type, *input.veh_geo_params, *input.psmm,
                            *input.pose, parking_spot_info,
                            state->path_manager_state().goal()));

  // ----------------------------------------------------------
  // -------------------- Freespace Planner -------------------
  // ----------------------------------------------------------
  FreespacePlannerInput freespace_input{
      .new_task = input.reset,
      .ego_pose = input.pose,
      .coordinate_converter = input.coordinate_converter,
      .chassis = input.chassis,
      .veh_geo_params = input.veh_geo_params,
      .veh_drive_params = input.veh_drive_params,
      .planner_params = input.planner_params,
      .obj_mgr = input.object_manager,
      .psmm = input.psmm,
      .stalled_object_ids = &stalled_objects,
      .plan_start_point = &input.plan_start_point_info->start_point,
      .start_point_reset = input.plan_start_point_info->reset,
      .plan_time = input.plan_time,
      .freespace_map = &freespace_map,
      .freespace_param = &input.planner_params->planner_params()
                              .freespace_params_for_parking()};
  ASSIGN_OR_RETURN(
      const auto freespace_planner_output,
      RunFreespacePlanner(freespace_input, state, &result->debug_proto,
                          &result->chart_data, thread_pool));

  const auto driving_state = GetOffRoadDrivingState(
      state->path_manager_state(), input.plan_start_point_info->full_stop);
  const auto past_points = CreatePastPointsList(
      input.plan_time, input.plan_start_point_info->start_point,
      *input.prev_trajectory_proto, input.plan_start_point_info->reset);

  result->trajectory_info = CreateFreespaceTrajectoryProto(
      input.plan_time, freespace_planner_output.traj_points, past_points,
      freespace_planner_output.gear_position, driving_state,
      freespace_planner_output.low_speed_freespace);

  return absl::OkStatus();
}

}  // namespace

absl::Status RunParkingTask(const ParkingTaskInput &input,
                            FreespacePlannerStateProto *state,
                            ParkingTaskOutput *result,
                            ThreadPool *thread_pool) {
  const auto status = ParkingTaskMain(input, state, result, thread_pool);
  result->debug_proto.set_planner_status(status.ToString());
  if (!status.ok()) {
    QLOG(ERROR) << "Freespace planner fails: " << status.ToString();
    // ----------------------------------------------------------
    // --------------------- AEB Planner-------------------------
    // ----------------------------------------------------------
    constexpr double kStopDeceleration = -1.0;
    const bool forward =
        (input.chassis->gear_location() != Chassis::GEAR_REVERSE);
    const auto output_traj = aeb::GenerateStopTrajectory(
        input.plan_start_point_info->path_s_increment_from_previous_frame,
        input.plan_start_point_info->reset, forward, kStopDeceleration,
        input.planner_params->planner_params().motion_constraint_params(),
        TrajectoryPoint(input.plan_start_point_info->start_point),
        {input.prev_trajectory_proto->trajectory_point().begin(),
         input.prev_trajectory_proto->trajectory_point().end()});
    const auto gear_position = input.chassis->gear_location();
    DrivingStateProto driving_state;
    driving_state.set_type(DrivingStateProto::STOPPED_WHEN_PARKING);
    const auto past_points = CreatePastPointsList(
        input.plan_time, input.plan_start_point_info->start_point,
        *input.prev_trajectory_proto, input.plan_start_point_info->reset);

    result->trajectory_info = CreateFreespaceTrajectoryProto(
        input.plan_time, output_traj, past_points, gear_position, driving_state,
        /*low_speed_freespace=*/false);
  }
  return absl::OkStatus();
}

}  // namespace  qcraft::planner
