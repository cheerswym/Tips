#include "onboard/planner/freespace/freespace_planner.h"

#include <utility>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/freespace/freespace_constraint_builder.h"
#include "onboard/planner/freespace/path_manager.h"
#include "onboard/planner/freespace/tob_path_smoother.h"
#include "onboard/planner/object/freespace_region_filter.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/path_util.h"

namespace qcraft::planner {

namespace {
// NOTE: Functions below are copied from freespace_planner.cc.

std::vector<ApolloTrajectoryPointProto>
GenerateStationaryTrajectoryWithRefKappa(const PathPoint &path_point,
                                         double ref_kappa, bool is_forward) {
  PathPoint start_path_point = path_point;

  double start_point_theta = start_path_point.theta();
  if (!is_forward) {
    start_point_theta = NormalizeAngle(start_point_theta + M_PI);
    ref_kappa = -ref_kappa;
  }
  start_path_point.set_kappa(ref_kappa);
  start_path_point.set_theta(start_point_theta);
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    ApolloTrajectoryPointProto traj_point;
    *traj_point.mutable_path_point() = start_path_point;
    traj_point.set_relative_time(i * kTrajectoryTimeStep);
    traj_points.push_back(std::move(traj_point));
  }
  return traj_points;
}

// TODO(opt): Pass by end parking gear instead of task type.
Chassis::GearPosition GenerateOutputGearPosition(
    const PathManagerOutput &path_output,
    const PathManagerStateProto &path_mgr_state) {
  Chassis::GearPosition gear_position;
  switch (path_mgr_state.drive_state()) {
    case PathManagerStateProto::UNKNOWN: {
      QLOG(FATAL) << "Unexpected UNKNOWN state.";
    }
    case PathManagerStateProto::SWITCHING_TO_NEXT: {
      gear_position = path_output.path.forward ? Chassis::GEAR_DRIVE
                                               : Chassis::GEAR_REVERSE;
      break;
    }
    case PathManagerStateProto::DRIVING: {
      gear_position = path_output.path.forward ? Chassis::GEAR_DRIVE
                                               : Chassis::GEAR_REVERSE;
      break;
    }
    case PathManagerStateProto::REACH_FINAL_GOAL:
    case PathManagerStateProto::CENTER_STEER: {
      switch (path_mgr_state.task_type()) {
        case FreespaceTaskProto::UNKNOWN_TASK:
          QLOG(FATAL) << "Unexpected UNKNOWN_TASK.";
        case FreespaceTaskProto::PERPENDICULAR_PARKING:
        case FreespaceTaskProto::PARALLEL_PARKING:
          gear_position = Chassis::GEAR_PARKING;
          break;
        case FreespaceTaskProto::THREE_POINT_TURN:
        case FreespaceTaskProto::DRIVING_TO_LANE:
        case FreespaceTaskProto::FREE_DRIVING:
          gear_position = Chassis::GEAR_DRIVE;
          break;
      }
      break;
    }
  }
  return gear_position;
}

absl::StatusOr<DirectionalPath> ExtendLocalPath(
    const DirectionalPath &directional_path) {
  if (directional_path.path.empty()) {
    return absl::InternalError("Discretized path empty.");
  }
  std::vector<PathPoint> raw_path_points = {directional_path.path.begin(),
                                            directional_path.path.end()};
  constexpr double kExtendPathLength = 5.0;         // m.
  constexpr double kExtendPathPointIntervel = 0.1;  // m.
  raw_path_points.reserve(
      raw_path_points.size() +
      CeilToInt(kExtendPathLength / kExtendPathPointIntervel));
  const auto &last_path_point = directional_path.path.back();
  for (double s = kExtendPathPointIntervel; s < kExtendPathLength;
       s += kExtendPathPointIntervel) {
    raw_path_points.push_back(GetPathPointAlongCircle(last_path_point, s));
  }
  // Resample raw path to final path.
  DiscretizedPath raw_path(std::move(raw_path_points));
  constexpr double kPathSampleInterval = 0.2;  // m.
  double s = 0.0;
  std::vector<PathPoint> path_points;
  path_points.reserve(CeilToInt(raw_path.length() / kPathSampleInterval));
  while (s < raw_path.length()) {
    path_points.push_back(raw_path.Evaluate(s));
    s += kPathSampleInterval;
  }

  DirectionalPath extend_directional_path;
  extend_directional_path.path = DiscretizedPath(std::move(path_points));
  extend_directional_path.forward = directional_path.forward;

  return extend_directional_path;
}

absl::StatusOr<std::vector<ApolloTrajectoryPointProto>>
GenerateOutputTrajectory(
    const PathManagerOutput &path_manager_output,
    const PathManagerStateProto &path_mgr_state,
    const DirectionalPath &smooth_directional_path,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const PlannerSemanticMapManager &psmm,
    const ConstraintManager &constraint_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geometry_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    const MotionConstraintParamsProto &motion_constraint_params,
    const SpeedFinderParamsProto &speed_finder_params,
    SpeedFinderDebugProto *speed_finder_debug,
    vis::vantage::ChartsDataProto *charts_data, ThreadPool *thread_pool) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  switch (path_mgr_state.drive_state()) {
    case PathManagerStateProto::UNKNOWN: {
      QLOG(FATAL) << "Unexpected UNKNOWN state.";
    }
    case PathManagerStateProto::SWITCHING_TO_NEXT: {
      traj_points = GenerateStationaryTrajectoryWithRefKappa(
          smooth_directional_path.path.front(),
          path_manager_output.path.path.front().kappa(),
          smooth_directional_path.forward);
      break;
    }
    case PathManagerStateProto::DRIVING: {
      // Run a speed finder on the local smooth path.
      FreespaceSpeedFinderInput speed_input;
      speed_input.semantic_map_manager = psmm.semantic_map_manager();
      speed_input.obj_mgr = &st_traj_mgr;
      speed_input.constraint_mgr = &constraint_mgr;
      speed_input.stalled_objects = &stalled_objects;
      speed_input.path = &smooth_directional_path.path;
      speed_input.forward = smooth_directional_path.forward;
      speed_input.plan_start_point = plan_start_point;
      ASSIGN_OR_RETURN(
          auto speed_output,
          FindFreespaceSpeed(speed_input, vehicle_geometry_params,
                             vehicle_drive_params, motion_constraint_params,
                             speed_finder_params, thread_pool));
      traj_points = std::move(speed_output.trajectory_points);
      charts_data->add_charts()->Swap(&speed_output.st_graph_chart);
      charts_data->add_charts()->Swap(&speed_output.vt_graph_chart);
      charts_data->add_charts()->Swap(&speed_output.traj_chart);
      speed_finder_debug->CopyFrom(speed_output.speed_finder_proto);
      break;
    }
    case PathManagerStateProto::REACH_FINAL_GOAL:
    case PathManagerStateProto::CENTER_STEER: {
      // Note: In `CENTER_STEER` and `REACH_FINAL_GOAL` state, current path is
      // the last path segment.
      traj_points = GenerateStationaryTrajectoryWithRefKappa(
          smooth_directional_path.path.front(), /*ref_kappa=*/0.0,
          smooth_directional_path.forward);
      break;
    }
  }
  return traj_points;
}

void FillFreespaceMapDebugProto(const FreespaceMap &freespace_map,
                                FreespaceMapDebugProto *debug_info) {
  QCHECK_NOTNULL(debug_info);
  debug_info->mutable_region()->Clear();
  debug_info->add_region(freespace_map.region.min_x());
  debug_info->add_region(freespace_map.region.min_y());
  debug_info->add_region(freespace_map.region.max_x());
  debug_info->add_region(freespace_map.region.max_y());
  debug_info->mutable_boundaries()->Clear();
  for (const auto &boundary : freespace_map.boundaries) {
    auto seg_proto = debug_info->add_boundaries();
    seg_proto->set_id(boundary.id);
    seg_proto->set_type(boundary.type);
    Vec2dToProto(boundary.segment.start(), seg_proto->mutable_start());
    Vec2dToProto(boundary.segment.end(), seg_proto->mutable_end());
  }
  for (const auto &boundary : freespace_map.special_boundaries) {
    if (boundary.type == SpecialBoundaryType::CROSSABLE_LANE_LINE) {
      continue;
    }
    auto seg_proto = debug_info->add_special_boundaries();
    seg_proto->set_id(boundary.id);
    Vec2dToProto(boundary.segment.start(), seg_proto->mutable_start());
    Vec2dToProto(boundary.segment.end(), seg_proto->mutable_end());
  }
}

bool CheckLowSpeed(
    bool forward, const MotionConstraintParamsProto &motion_constraint_params) {
  constexpr double kLowSpeedThreshold = 3.91;  // ~6.3km/h
  return !forward ||
         motion_constraint_params.default_speed_limit() < kLowSpeedThreshold;
}

}  // namespace

absl::StatusOr<FreespacePlannerOutput> RunFreespacePlanner(
    const FreespacePlannerInput &input, FreespacePlannerStateProto *state,
    FreespacePlannerDebugProto *debug_info,
    vis::vantage::ChartsDataProto *charts_data, ThreadPool *thread_pool) {
  SCOPED_QTRACE("FreespacePlanner");
  const auto start_time = absl::Now();

  const auto &freespace_map = *input.freespace_map;
  FillFreespaceMapDebugProto(freespace_map,
                             debug_info->mutable_freespace_map_debug());

  // TODO(renjie): Replace with BuildSpacetimeTrajectoryManager.
  AABox2d filter_region(
      freespace_map.region.half_length() + input.veh_geo_params->length(),
      freespace_map.region.half_width() + input.veh_geo_params->length(),
      freespace_map.region.center());
  const FreespaceRegionFilter freespace_region_filter(&filter_region);
  SpacetimeTrajectoryManager st_traj_mgr(
      /*filters=*/{&freespace_region_filter}, /*finders=*/{},
      input.obj_mgr->planner_objects(),
      /*st_planner_trajectories_start_offset=*/0.0, thread_pool);

  // Construct path manager.
  PathManager path_mgr(state->path_manager_state());

  const auto &stalled_object_ids = *input.stalled_object_ids;
  std::vector<const SpacetimeObjectTrajectory *> stalled_object_trajs;
  stalled_object_trajs.reserve(st_traj_mgr.trajectories().size());
  for (const auto traj : st_traj_mgr.trajectories()) {
    if (stalled_object_ids.contains(traj->object_id())) {
      stalled_object_trajs.push_back(traj);
    }
  }

  // TODO(weijun): min dependency in planner_params.
  ASSIGN_OR_RETURN(
      const auto path_output,
      path_mgr.GeneratePath(
          input.new_task, *input.ego_pose, *input.coordinate_converter,
          *input.chassis, input.freespace_param->hybrid_a_star_params(),
          *input.veh_geo_params, *input.veh_drive_params, stalled_object_trajs,
          freespace_map, input.plan_time, state->mutable_path_manager_state(),
          debug_info->mutable_ha_star_debug(),
          debug_info->mutable_path_manager_debug()));

  // Reset plan start point on receiving a new path. This can happen when path
  // finder replans, or path manager switches to next directional path.
  const auto &plan_start_point = *input.plan_start_point;
  const auto start_point =
      path_output.is_new_path
          ? ComputePlanStartPointAfterReset(
                std::optional<ApolloTrajectoryPointProto>(plan_start_point),
                *input.ego_pose, *input.chassis,
                input.planner_params->planner_params()
                    .motion_constraint_params(),
                *input.veh_geo_params, *input.veh_drive_params)
          : *input.plan_start_point;

  ASSIGN_OR_RETURN(
      const auto smooth_directional_path,
      SmoothLocalPath(*input.veh_geo_params, *input.veh_drive_params,
                      input.freespace_param->motion_constraint_params(),
                      input.freespace_param->local_smoother_params(),
                      input.planner_params->planner_params()
                          .trajectory_optimizer_vehicle_model_params(),
                      /*owner=*/"freespace_local_smoother", freespace_map,
                      st_traj_mgr, stalled_object_ids, path_output.path,
                      TrajectoryPoint(*input.plan_start_point),
                      (path_output.is_new_path || input.start_point_reset),
                      {state->prev_local_smoother_path().begin(),
                       state->prev_local_smoother_path().end()},
                      debug_info->mutable_local_smoother_debug(), charts_data));

  *state->mutable_prev_local_smoother_path() = {
      smooth_directional_path.path.begin(), smooth_directional_path.path.end()};

  ASSIGN_OR_RETURN(auto constraint_mgr,
                   BuildFreespacePlannerConstraint(*input.veh_geo_params,
                                                   smooth_directional_path));

  UpdateDecisionConstraintDebugInfo(constraint_mgr,
                                    debug_info->mutable_decision_constraint());

  ASSIGN_OR_RETURN(const auto extend_smooth_directional_path,
                   ExtendLocalPath(smooth_directional_path));

  ASSIGN_OR_RETURN(
      auto trajectory,
      GenerateOutputTrajectory(
          path_output, state->path_manager_state(),
          extend_smooth_directional_path, st_traj_mgr, *input.psmm,
          constraint_mgr, stalled_object_ids, *input.plan_start_point,
          *input.veh_geo_params, *input.veh_drive_params,
          input.freespace_param->motion_constraint_params(),
          input.freespace_param->speed_finder_params(),
          debug_info->mutable_speed_finder_debug(), charts_data, thread_pool));
  const auto freespace_plan_time =
      absl::ToDoubleMilliseconds(absl::Now() - start_time);
  constexpr double kFreespacePlaneTimeoutThresholdMs = 100.0;  // ms.
  if (freespace_plan_time > kFreespacePlaneTimeoutThresholdMs) {
    QEVENT("yuhang", "freespace_plan_timeout", [&](QEvent *qevent) {
      qevent->AddField("freespace_plan_time[ms]", freespace_plan_time)
          .AddField("time_limit[ms]", kFreespacePlaneTimeoutThresholdMs);
    });
  }

  return FreespacePlannerOutput{
      .traj_points = std::move(trajectory),
      .gear_position =
          GenerateOutputGearPosition(path_output, state->path_manager_state()),
      .low_speed_freespace =
          CheckLowSpeed(extend_smooth_directional_path.forward,
                        input.freespace_param->motion_constraint_params())};
}

};  // namespace qcraft::planner
