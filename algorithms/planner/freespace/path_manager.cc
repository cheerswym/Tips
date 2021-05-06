#include "onboard/planner/freespace/path_manager.h"

#include <algorithm>
#include <string>

#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"
#include "onboard/planner/freespace/ipopt_segmented_global_path_smoother.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/time_util.h"

DEFINE_bool(freespace_planner_smooth_global_path, false,
            "Whether to smooth the global path.");

namespace qcraft {
namespace planner {
namespace {

PathPoint ComputeReplanStartPoint(
    const PoseProto &pose, const Chassis &chassis,
    const VehicleGeometryParamsProto &vehicle_geom_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  PathPoint start_point;
  start_point.set_x(pose.pos_smooth().x());
  start_point.set_y(pose.pos_smooth().y());
  start_point.set_s(0.0);
  start_point.set_theta(pose.yaw());
  const double pose_v = pose.vel_body().x();

  // At speed lower than this, we don't trust the measured acceleration and
  // angular velocity. Reset like this are mostly when we're stopped anyway.
  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (pose_v < kLowSpeedThreshold) {
    const double steer_angle =
        chassis.has_steering_percentage()
            ? chassis.steering_percentage() * 0.01 *
                  vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()
            : 0.0;  // rad.
    const double kappa =
        std::tan(steer_angle) / vehicle_geom_params.wheel_base();
    start_point.set_kappa(kappa);
    start_point.set_lambda(0.0);
  } else {
    start_point.set_kappa(pose.ar_smooth().z() / pose_v);
    start_point.set_lambda(0.0);
  }
  return start_point;
}

void AddHybridAStarPathToDebug(const std::vector<DirectionalPath> &paths,
                               HybridAStartDebugProto *debug_info) {
  for (const auto &path : paths) {
    DirectionalPathProto *path_proto = debug_info->add_paths();
    path_proto->set_forward(path.forward);
    for (const auto &pt : path.path) {
      path_proto->add_path()->CopyFrom(pt);
    }
  }
}

void ConvertPathsToGlobalCoordinates(
    const CoordinateConverter &coordinate_converter,
    std::vector<DirectionalPath> paths,
    std::vector<std::vector<Vec2d>> *global_pos,
    std::vector<std::vector<double>> *global_theta) {
  global_pos->clear();
  global_theta->clear();
  global_pos->reserve(paths.size());
  global_theta->reserve(paths.size());
  for (const auto &dir_path : paths) {
    auto &pos = global_pos->emplace_back();
    auto &theta = global_theta->emplace_back();
    pos.reserve(dir_path.path.size());
    theta.reserve(dir_path.path.size());
    for (const auto &p : dir_path.path) {
      pos.push_back(coordinate_converter.SmoothToGlobal(ToVec2d(p)));
      theta.push_back(
          coordinate_converter.SmoothYawToGlobalNoNormalize(p.theta()));
    }
  }
}

void ConvertPathsToCurrentSmooth(
    const CoordinateConverter &coordinate_converter,
    std::vector<std::vector<Vec2d>> global_pos,
    std::vector<std::vector<double>> global_theta,
    std::vector<DirectionalPath> *paths) {
  QCHECK_EQ(global_pos.size(), global_theta.size());
  QCHECK_EQ(global_pos.size(), paths->size());
  for (int i = 0; i < paths->size(); ++i) {
    QCHECK_EQ(global_pos[i].size(), (*paths)[i].path.size());
    QCHECK_EQ(global_theta[i].size(), (*paths)[i].path.size());
    for (int j = 0; j < (*paths)[i].path.size(); ++j) {
      auto &path_point = (*paths)[i].path[j];
      const Vec2d smooth_pos =
          coordinate_converter.GlobalToSmooth(global_pos[i][j]);
      const double smooth_yaw =
          coordinate_converter.GlobalYawToSmoothNoNormalize(global_theta[i][j]);
      path_point.set_x(smooth_pos.x());
      path_point.set_y(smooth_pos.y());
      path_point.set_theta(smooth_yaw);
    }
  }
}

}  // namespace

PathManager::PathManager(const PathManagerStateProto &state)
    : goal_(state.goal()),
      global_goal_(state.global_goal()),
      task_type_(state.task_type()),
      curr_path_idx_(state.curr_path_idx()),
      drive_state_(state.drive_state()) {
  paths_.reserve(state.paths_size());
  paths_global_pos_.reserve(state.paths_size());
  paths_global_theta_.reserve(state.paths_size());
  for (int i = 0; i < state.paths_size(); ++i) {
    DirectionalPath path;
    path.FromProto(state.paths(i));
    paths_.push_back(std::move(path));
    auto &global_pos = paths_global_pos_.emplace_back();
    global_pos.reserve(state.paths_global_pos(i).pos_size());
    for (int j = 0; j < state.paths_global_pos(i).pos_size(); ++j) {
      global_pos.emplace_back(state.paths_global_pos(i).pos(j));
    }
    auto &global_theta = paths_global_theta_.emplace_back();
    global_theta.reserve(state.paths_global_theta(i).theta_size());
    for (int j = 0; j < state.paths_global_theta(i).theta_size(); ++j) {
      global_theta.push_back(state.paths_global_theta(i).theta(j));
    }
  }
}

absl::StatusOr<PathManagerOutput> PathManager::GeneratePath(
    bool new_task, const PoseProto &ego_pose,
    const CoordinateConverter &coordinate_converter, const Chassis &chassis,
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &veh_drive_params,
    absl::Span<const SpacetimeObjectTrajectory *const> stalled_object_trajs,
    const FreespaceMap &freespace_map, absl::Time unused_plan_time,
    PathManagerStateProto *path_mgr_state,
    HybridAStartDebugProto *ha_star_debug,
    PathManagerDebugProto *path_mgr_debug) {
  SCOPED_QTRACE("GeneratePath");

  // Whether we should start a new path finding process.
  bool start_new_plan = false;
  if (new_task) {
    // Check the validity of the new goal.
    if (false /*!CheckGoal(goal_)*/) {
      QLOG(WARNING) << "Refuse to generate path because goal is invalid.";
      *path_mgr_state = MakeState();
      return absl::InternalError("Invalid goal!");
    }
    start_new_plan = true;
  }

  if (!start_new_plan) {
    bool replan = false;
    // Check if need to do a replan. Can wrap the checks into a
    // function later.
    // 1. If we have a valid goal but haven't a plan yet, replan.
    if (paths_.empty()) {
      replan = true;
      path_mgr_debug->add_replan_reason(PathManagerDebugProto::NO_PATHS);
    }
    // TODO(yumeng): Add pre-filter of stationary_object_trajs.
    if (!paths_.empty()) {
      const auto path_safety = PathSafetyCheck(
          veh_geo_params, stalled_object_trajs, paths_, curr_path_idx_);
      if (!path_safety.ok()) {
        replan = true;
        path_mgr_debug->add_replan_reason(PathManagerDebugProto::PATH_NOT_SAFE);
      }
    }

    // 2. If the global path is blocked by newly detected objects, replan.
    // TODO(yumeng): Implement this check.
    // 3. If AV has stopped too far from a path segment end due to
    // control error, replan.
    // TODO(yumeng): Implement this check.

    start_new_plan = replan;
  }

  bool is_new_path = false;
  if (start_new_plan) {
    const auto start_point = ComputeReplanStartPoint(
        ego_pose, chassis, veh_geo_params, veh_drive_params);
    if (const auto paths =
            FindPath(hybrid_a_star_params, veh_geo_params, veh_drive_params,
                     task_type_, freespace_map, stalled_object_trajs,
                     start_point, goal_, ha_star_debug);
        paths.ok()) {
      paths_ = std::move(*paths);
    } else {
      *path_mgr_state = MakeState();
      return paths.status();
    }

    // Smooth the coarse path globally into denser second-order continuous path.
    // The result is not used now.
    if (FLAGS_freespace_planner_smooth_global_path) {
      const auto smooth_paths = SmoothGlobalPath(
          paths_, freespace_map, veh_geo_params, veh_drive_params);
      if (!smooth_paths.ok()) {
        QLOG(WARNING) << "Segmented global path smoother fails: "
                      << smooth_paths.status();
      }
    }

    // Convert and store the global coordinates of the paths.
    ConvertPathsToGlobalCoordinates(coordinate_converter, paths_,
                                    &paths_global_pos_, &paths_global_theta_);

    // Reset some states.
    // paths_ = AnalyzeGlobalPath(smooth_path);
    curr_path_idx_ = 0;
    drive_state_ = PathManagerStateProto::SWITCHING_TO_NEXT;
    is_new_path = true;
    // TODO(yumeng): Complete this.
  } else {
    // Convert paths to current smooth origin if there is no new plan.
    ConvertPathsToCurrentSmooth(coordinate_converter, paths_global_pos_,
                                paths_global_theta_, &paths_);
  }

  bool switched_to_new_path = false;
  UpdatePathManagerState(veh_geo_params, veh_drive_params, task_type_, ego_pose,
                         chassis, paths_, &drive_state_, &curr_path_idx_,
                         &switched_to_new_path);
  is_new_path |= switched_to_new_path;
  AddHybridAStarPathToDebug(paths_, ha_star_debug);
  path_mgr_debug->set_curr_path_idx(curr_path_idx_);
  path_mgr_debug->set_drive_state(drive_state_);
  *path_mgr_state = MakeState();
  return PathManagerOutput(
      {.path = paths_[curr_path_idx_], .is_new_path = is_new_path});
}

}  // namespace planner
}  // namespace qcraft
