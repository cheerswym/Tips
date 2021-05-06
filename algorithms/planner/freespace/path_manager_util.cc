#include "onboard/planner/freespace/path_manager_util.h"

#include <algorithm>
#include <utility>

#include "gflags/gflags.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

// TODO(shijun, yumeng): Remove it after control precision is acceptable.
DEFINE_double(distance_threshold, 0.3,
              "Judge av reach goal point distance thresold, unit: m");

namespace qcraft {
namespace planner {
namespace {

inline bool IsAvFullStop(double speed) {
  constexpr double kSpeedThreshold = 0.1;  // m/s.
  return std::abs(speed) < kSpeedThreshold;
}

inline bool IsGoalReached(const DirectionalPath &path,
                          const PoseProto &av_pose) {
  const Vec2d av_pos(av_pose.pos_smooth().x(), av_pose.pos_smooth().y());
  const auto current_sl = path.path.XYToSL(av_pos);
  const double distance_to_goal = path.path.back().s() - current_sl.s;
  VLOG(1) << "Distance to goal: " << distance_to_goal << "m";
  if (distance_to_goal < FLAGS_distance_threshold) {
    return true;
  }
  return false;
}

inline bool IsGearShiftedToRefDirection(
    const Chassis::GearPosition &gear_position, bool is_next_path_forward) {
  return (is_next_path_forward && gear_position == Chassis::GEAR_DRIVE) ||
         (!is_next_path_forward && gear_position == Chassis::GEAR_REVERSE);
}

inline double Kappa2FrontWheelAngle(double kappa, double wheel_base) {
  return std::atan(kappa * wheel_base);
}

inline double GetFrontWheelAngle(double steering_percentage,
                                 double max_steer_angle, double steer_ratio) {
  return steering_percentage * 0.01 * max_steer_angle / steer_ratio;  // rad.
}

inline bool IsSteerAngleReached(double steering_wheel_speed, double steer_angle,
                                double target_steer_angle) {
  // TODO(shijun): Modify this value from control side.
  constexpr double kSteerAngleThreshold = 0.1;          // rad.
  constexpr double kSteeringWheelSpeedThreshold = 4.0;  // deg/s.
  return std::abs(target_steer_angle - steer_angle) < kSteerAngleThreshold &&
         steering_wheel_speed < kSteeringWheelSpeedThreshold;
}

// Currently freespace planner has two type tasks: 1. Stop at a place we
// want. 2. Jointly driving with other planner. So gear should be GEAR_DRIVE
// or GEAR_PARKING.
inline bool IsGearMatchedTask(const Chassis::GearPosition &gear_position,
                              const FreespaceTaskProto::TaskType &task_type) {
  bool is_matched = false;
  switch (task_type) {
    case FreespaceTaskProto::UNKNOWN_TASK:
      QLOG(FATAL) << "Unexpected UNKNOWN_TASK.";
    case FreespaceTaskProto::PERPENDICULAR_PARKING:
    case FreespaceTaskProto::PARALLEL_PARKING: {
      if (gear_position == Chassis::GEAR_PARKING) {
        is_matched = true;
      }
      break;
    }
    case FreespaceTaskProto::THREE_POINT_TURN:
    case FreespaceTaskProto::DRIVING_TO_LANE:
    case FreespaceTaskProto::FREE_DRIVING: {
      if (gear_position == Chassis::GEAR_DRIVE) {
        is_matched = true;
      }
      break;
    }
  }
  return is_matched;
}

inline bool IsSteeringWheelBackToCenter(
    const VehicleGeometryParamsProto &vehicle_geom,
    const VehicleDriveParamsProto &vehicle_drive, double steering_wheel_speed,
    double steering_percentage, const PathPoint &av_path_point) {
  const double steer_angle =
      GetFrontWheelAngle(steering_percentage, vehicle_drive.max_steer_angle(),
                         vehicle_drive.steer_ratio());
  return IsSteerAngleReached(steering_wheel_speed, steer_angle,
                             /*target_steer_angle=*/0.0);
}
}  // namespace

void UpdatePathManagerState(const VehicleGeometryParamsProto &vehicle_geom,
                            const VehicleDriveParamsProto &vehicle_drive,
                            const FreespaceTaskProto::TaskType &task_type,
                            const PoseProto &av_pose, const Chassis &chassis,
                            absl::Span<const DirectionalPath> paths,
                            DriveState *drive_state, int *curr_path_idx,
                            bool *switched_to_new_path) {
  *switched_to_new_path = false;
  QCHECK_LT(*curr_path_idx, paths.size());
  VLOG(2) << "drive_state: " << *drive_state;

  switch (*drive_state) {
    case PathManagerStateProto::UNKNOWN: {
      QLOG(FATAL) << "Unexpected UNKNOWN state.";
    }
    case PathManagerStateProto::DRIVING: {
      const PathPoint &current_goal = paths[*curr_path_idx].path.back();
      const bool is_last_path_segment =
          (*curr_path_idx == paths.size() - 1) ? true : false;
      const bool is_av_full_stop = IsAvFullStop(av_pose.vel_body().x());
      const bool is_goal_reached =
          IsGoalReached(paths[*curr_path_idx], av_pose);
      if (is_av_full_stop && is_goal_reached) {
        if (is_last_path_segment) {
          if (IsSteeringWheelBackToCenter(
                  vehicle_geom, vehicle_drive, chassis.steering_speed(),
                  chassis.steering_percentage(), current_goal) &&
              IsGearMatchedTask(chassis.gear_location(), task_type)) {
            *drive_state = PathManagerStateProto::REACH_FINAL_GOAL;
          } else {
            *drive_state = PathManagerStateProto::CENTER_STEER;
          }
        } else {
          *curr_path_idx = *curr_path_idx + 1;
          *drive_state = PathManagerStateProto::SWITCHING_TO_NEXT;
          *switched_to_new_path = true;
        }
      }
    } break;
    case PathManagerStateProto::SWITCHING_TO_NEXT: {
      const PathPoint &current_start_point = paths[*curr_path_idx].path.front();
      double current_start_kappa = current_start_point.kappa();
      if (!paths[*curr_path_idx].forward) {
        current_start_kappa = -current_start_kappa;
      }
      const double target_front_wheel_angle =
          Kappa2FrontWheelAngle(current_start_kappa, vehicle_geom.wheel_base());
      const double front_wheel_angle = GetFrontWheelAngle(
          chassis.steering_percentage(), vehicle_drive.max_steer_angle(),
          vehicle_drive.steer_ratio());
      const bool is_current_forward = paths[*curr_path_idx].forward;
      const bool is_steer_angle_reached =
          IsSteerAngleReached(chassis.steering_speed(), front_wheel_angle,
                              target_front_wheel_angle);
      const bool is_gear_shifted = IsGearShiftedToRefDirection(
          chassis.gear_location(), is_current_forward);

      VLOG(2) << "Target front wheel angle: " << target_front_wheel_angle
              << ". Front wheel angle: " << front_wheel_angle;
      VLOG(2) << "Steer angle reached: "
              << (is_steer_angle_reached ? "true" : "false");
      VLOG(2) << "Gear shifted: " << (is_gear_shifted ? "true" : "false");

      if (is_steer_angle_reached && is_gear_shifted) {
        *drive_state = PathManagerStateProto::DRIVING;
      }
    } break;
    case PathManagerStateProto::CENTER_STEER: {
      const PathPoint &current_goal = paths[*curr_path_idx].path.back();
      if (IsSteeringWheelBackToCenter(
              vehicle_geom, vehicle_drive, chassis.steering_speed(),
              chassis.steering_percentage(), current_goal) &&
          IsGearMatchedTask(chassis.gear_location(), task_type)) {
        *drive_state = PathManagerStateProto::REACH_FINAL_GOAL;
      }
    } break;
    case PathManagerStateProto::REACH_FINAL_GOAL:
      break;
  }
}

std::vector<Box2d> PathSweptVolume(
    const VehicleGeometryParamsProto &vehicle_geom,
    absl::Span<const DirectionalPath *const> paths, const double length_buffer,
    const double width_buffer) {
  std::vector<Box2d> path_swept_volume;
  if (paths.empty()) return path_swept_volume;
  if (paths.front()->path.empty()) return path_swept_volume;

  const auto &first_path_point = paths.front()->path.front();
  const auto av_pos = Vec2d(first_path_point.x(), first_path_point.y());

  // Theta towards vehicle head.
  double av_regularized_theta = first_path_point.theta();
  if (!paths.front()->forward) {
    av_regularized_theta = NormalizeAngle(first_path_point.theta() + M_PI);
  }

  const auto av_box = GetAvBoxWithBuffer(
      av_pos, av_regularized_theta, vehicle_geom, length_buffer, width_buffer);
  const auto &av_box_center = av_box.center();
  const double rac_to_center =
      vehicle_geom.length() * 0.5 - vehicle_geom.back_edge_to_center();

  int path_point_size = 0;
  for (const auto directional_path : paths) {
    path_point_size += directional_path->path.size();
  }

  path_swept_volume.reserve(path_point_size);
  for (const auto directional_path : paths) {
    const auto &path_points = directional_path->path;
    for (int i = 0; i < path_points.size(); ++i) {
      double regularized_theta = path_points[i].theta();
      if (!directional_path->forward) {
        regularized_theta = NormalizeAngle(regularized_theta + M_PI);
      }
      // Rotation
      const double rotation = regularized_theta - av_regularized_theta;
      // Translation
      const Vec2d tr_pos(path_points[i].x(), path_points[i].y());
      const Vec2d tangent = Vec2d::FastUnitFromAngle(regularized_theta);
      const Vec2d tr_center = tr_pos + tangent * rac_to_center;
      const Vec2d translation = tr_center - av_box_center;

      Box2d tr_box = av_box.Transform(translation, rotation);
      path_swept_volume.push_back(std::move(tr_box));
    }
  }
  return path_swept_volume;
}

absl::StatusOr<PathPoint> GetPathPointFromGlobalIndex(
    absl::Span<const DirectionalPath *const> paths, const int global_index) {
  if (global_index < 0) return absl::InternalError("global_index is negative.");

  int count = 0;
  for (int i = 0; i < paths.size(); ++i) {
    count += paths[i]->path.size();
    if (count >= global_index) {
      int current_seg_index = global_index - (count - paths[i]->path.size());
      return paths[i]->path[current_seg_index];
    }
  }
  return absl::InternalError("global_index overflow.");
}

// This path safety check just check global paths after current driving
// segment.
absl::Status PathSafetyCheck(
    const VehicleGeometryParamsProto &vehicle_geom,
    absl::Span<const SpacetimeObjectTrajectory *const> stalled_object_trajs,
    absl::Span<const DirectionalPath> paths, const int current_index) {
  if (paths.empty()) {
    return absl::OkStatus();
  }
  // No check if global path remain current segment.
  if (current_index == paths.size() - 1) {
    return absl::OkStatus();
  }
  std::vector<const DirectionalPath *> global_paths_remain;
  global_paths_remain.reserve(paths.size() - current_index - 1);
  for (int i = current_index + 1; i < paths.size(); ++i) {
    global_paths_remain.push_back(&paths[i]);
  }
  constexpr double kLengthBuffer = 0.25;
  constexpr double kWidthBuffer = 0.25;
  const auto path_swept_volume = PathSweptVolume(
      vehicle_geom, global_paths_remain, kLengthBuffer, kWidthBuffer);
  if (path_swept_volume.empty()) {
    return absl::InternalError("Path swept volume empty.");
  }

  // TODO(yumeng): Consider use parallel calculation.
  for (const auto stationary_object_traj : stalled_object_trajs) {
    const auto &contour = stationary_object_traj->planner_object()->contour();
    for (int i = 0; i < path_swept_volume.size(); ++i) {
      if (contour.HasOverlap(path_swept_volume[i])) {
        const auto path_point =
            GetPathPointFromGlobalIndex(global_paths_remain, i);
        QCHECK(path_point.ok()) << path_point.status().ToString();
        return absl::InternalError(absl::StrFormat(
            "For index: %d path point: (%f, %f, %f) has collision with "
            "stationary "
            "traj: %s.",
            i, path_point->x(), path_point->y(), path_point->theta(),
            stationary_object_traj->traj_id()));
      }
    }
  }
  return absl::OkStatus();
}

}  // namespace planner
}  // namespace qcraft
