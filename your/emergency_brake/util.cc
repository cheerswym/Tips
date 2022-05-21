#include "onboard/emergency_brake/util.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace {

constexpr double kSideCollisionAngleLimit = M_PI / 6.0;

bool IsBackOrSideCollision(const Box2d& av_box, const Box2d& obj_box) {
  if (std::fabs(NormalizeAngle(av_box.heading() - obj_box.heading())) >
      kSideCollisionAngleLimit) {
    return false;
  }
  const double proj =
      (obj_box.center() - av_box.RearCenterPoint()).dot(av_box.tangent());
  if (proj < av_box.half_length()) {
    return true;
  }
  return false;
}

}  // namespace
namespace emergency_brake {

absl::StatusOr<Polygon2d> ComputeCircularDetectionPolygon(
    const Box2d& av_box, double rac_to_center, double signed_drive_distance,
    double signed_circular_radius) {
  constexpr double kStepDistance = 0.25;
  const int num_steps =
      CeilToInt(std::abs(signed_drive_distance) / kStepDistance);
  if (num_steps <= 0) {
    return absl::FailedPreconditionError(
        absl::StrCat("Drive distance is invalid ", signed_drive_distance));
  }

  std::vector<Vec2d> left_corners;
  std::vector<Vec2d> right_corners;
  left_corners.reserve(num_steps);
  right_corners.reserve(num_steps * 2);

  // Find key points on the original AV box.
  const bool is_forward = signed_drive_distance > 0.0;
  const Vec2d origin_left =
      av_box.GetCorner(is_forward ? Box2d::FRONT_LEFT : Box2d::REAR_RIGHT);
  const Vec2d origin_right =
      av_box.GetCorner(is_forward ? Box2d::FRONT_RIGHT : Box2d::REAR_LEFT);

  const Vec2d origin_av_pos =
      av_box.center() - rac_to_center * av_box.tangent();
  const Vec2d rotation_center =
      origin_av_pos + signed_circular_radius * av_box.tangent().Perp();

  left_corners.push_back(origin_left);
  right_corners.push_back(origin_right);

  const double curvature = 1.0 / signed_circular_radius;
  const double adjusted_step_dist = signed_drive_distance / num_steps;
  double s = adjusted_step_dist;
  for (int i = 0; i < num_steps; ++i, s += adjusted_step_dist) {
    // Compute the RAC position when AV moved by s.
    const double rotation_angle = NormalizeAngle(s * curvature);
    const double sin_angle = fast_math::SinNormalized(rotation_angle);
    const double cos_angle = fast_math::CosNormalized(rotation_angle);

    const Vec2d left_pt =
        (origin_left - rotation_center).Rotate(cos_angle, sin_angle) +
        rotation_center;
    left_corners.push_back(left_pt);

    const Vec2d right_pt =
        (origin_right - rotation_center).Rotate(cos_angle, sin_angle) +
        rotation_center;
    right_corners.push_back(right_pt);
  }

  // Add the front left corners to front right corners in reverse order to form
  // the polygon's edge.
  std::vector<Vec2d> polygon_vertices = std::move(right_corners);
  for (int i = left_corners.size() - 1; i >= 0; --i) {
    polygon_vertices.push_back(left_corners[i]);
  }
  return Polygon2d(std::move(polygon_vertices));
}

std::vector<ApolloTrajectoryPointProto> ComputeConstJerkCircularMotion(
    const ApolloTrajectoryPointProto& start, Duration dt, Duration max_duration,
    Jerk jerk) {
  std::vector<ApolloTrajectoryPointProto> traj;
  traj.reserve(CeilToInt(ToNumber(max_duration / dt)) + 1);
  traj.push_back(start);

  const Acceleration init_a = MetersPerSquaredSecond(start.a());
  const Velocity init_v = MetersPerSecond(start.v());
  const Vec2d init_xy(start.path_point().x(), start.path_point().y());
  const Vec2d init_tangent =
      Vec2d::FastUnitFromAngle(start.path_point().theta());
  const Length rotation_radius =
      Meters((std::abs(start.path_point().kappa()) > 1e-8)
                 ? (1.0 / start.path_point().kappa())
                 : 1e8);
  const Vec2d rotation_center =
      init_xy + init_tangent.Perp() * ToMeters(rotation_radius);
  for (Duration t = dt;; t += dt) {
    constexpr double OneOverSix = 1.0 / 6.0;
    const Acceleration a = init_a + jerk * t;
    const Velocity v = init_v + (init_a + 0.5 * jerk * t) * t;
    const Length s = (init_v + (0.5 * init_a + OneOverSix * jerk * t) * t) * t;
    const double d_theta = ToNumber(s / rotation_radius);
    const Vec2d rotation = Vec2d::UnitFromAngle(d_theta);
    const Vec2d xy =
        (init_xy - rotation_center).Rotate(rotation.x(), rotation.y()) +
        rotation_center;

    ApolloTrajectoryPointProto proto;
    proto.set_relative_time(ToSeconds(t) + start.relative_time());
    proto.set_v(std::max(0.0, ToMetersPerSecond(v)));
    proto.set_a(ToMetersPerSquaredSecond(a));
    proto.set_j(ToMetersPerCubicSecond(jerk));
    proto.mutable_path_point()->set_x(xy.x());
    proto.mutable_path_point()->set_y(xy.y());
    proto.mutable_path_point()->set_z(0.0);
    proto.mutable_path_point()->set_theta(
        NormalizeAngle(start.path_point().theta() + d_theta));
    proto.mutable_path_point()->set_kappa(start.path_point().kappa());
    proto.mutable_path_point()->set_s(start.path_point().s() + ToMeters(s));
    proto.mutable_path_point()->set_lambda(0.0);
    traj.push_back(std::move(proto));

    if (v < Velocity::Zero() || t >= max_duration) break;
  }
  return traj;
}

std::vector<ApolloTrajectoryPointProto> ComputeConstAccelerationCircularMotion(
    const ApolloTrajectoryPointProto& start, Duration dt, Duration max_duration,
    Acceleration accel) {
  std::vector<ApolloTrajectoryPointProto> traj;
  traj.reserve(CeilToInt(ToNumber(max_duration / dt)) + 1);
  traj.push_back(start);

  const Velocity init_v = MetersPerSecond(start.v());
  const Vec2d init_xy(start.path_point().x(), start.path_point().y());
  const Vec2d init_tangent =
      Vec2d::FastUnitFromAngle(start.path_point().theta());
  const Length rotation_radius =
      Meters((std::abs(start.path_point().kappa()) > 1e-8)
                 ? (1.0 / start.path_point().kappa())
                 : 1e8);
  const Vec2d rotation_center =
      init_xy + init_tangent.Perp() * ToMeters(rotation_radius);
  for (Duration t = dt;; t += dt) {
    const Velocity v = init_v + accel * t;
    const Length s = (init_v + 0.5 * accel * t) * t;
    const double d_theta = ToNumber(s / rotation_radius);
    const Vec2d rotation = Vec2d::UnitFromAngle(d_theta);
    const Vec2d xy =
        (init_xy - rotation_center).Rotate(rotation.x(), rotation.y()) +
        rotation_center;

    ApolloTrajectoryPointProto proto;
    proto.set_relative_time(ToSeconds(t) + start.relative_time());
    proto.set_v(std::max(0.0, ToMetersPerSecond(v)));
    proto.set_a(ToMetersPerSquaredSecond(accel));
    proto.set_j(0.0);
    proto.mutable_path_point()->set_x(xy.x());
    proto.mutable_path_point()->set_y(xy.y());
    proto.mutable_path_point()->set_z(0.0);
    proto.mutable_path_point()->set_theta(
        NormalizeAngle(start.path_point().theta() + d_theta));
    proto.mutable_path_point()->set_kappa(start.path_point().kappa());
    proto.mutable_path_point()->set_s(start.path_point().s() + ToMeters(s));
    proto.mutable_path_point()->set_lambda(0.0);
    traj.push_back(std::move(proto));

    if (v < Velocity::Zero() || t >= max_duration) break;
  }
  return traj;
}

std::optional<FenDetectionProto> ComputeFenCollisions(
    const std::vector<ApolloTrajectoryPointProto>& traj, double traj_start_time,
    const VehicleGeometryParamsProto& vehicle_geom,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const FenDetectionsProto& fen_detections) {
  if (traj.empty()) {
    return std::nullopt;
  }
  if (fen_detections.detections_size() == 0) {
    return std::nullopt;
  }

  // Note(Jinyun): As how this function is currenly used,
  // it's assumed that av's current velocity is the begining velocity of traj.
  const double cur_v = traj[0].v();
  const double time_to_stop =
      std::abs(cur_v / vehicle_drive_params.max_deceleration());
  constexpr double kBufferTime = 0.5;
  const double traj_duration =
      traj.back().relative_time() - traj.front().relative_time();
  const double check_time = std::min(time_to_stop + kBufferTime, traj_duration);

  // Shift time to match the time sequence of traj and fen detection.
  const double obs_start_time = fen_detections.timestamp();
  double shift_time = traj_start_time - obs_start_time;
  int idx = 0;
  const int traj_size = traj.size();
  if (shift_time < 0) {
    while (traj[idx].relative_time() - traj.front().relative_time() <
           std::abs(shift_time)) {
      ++idx;
      if (idx >= traj_size) {
        return std::nullopt;
      }
    }
    shift_time =
        traj[idx].relative_time() - traj.front().relative_time() - shift_time;
  }
  const double start_relative_time = traj[idx].relative_time();

  constexpr double kLatExtendBuffer = 0.1;

  // Skip if AV is already overlapping with any fen detection.
  const auto& av_point = traj[0].path_point();
  const Box2d av_box = planner::GetAvBoxWithBuffer(
      Vec2d(av_point.x(), av_point.y()), av_point.theta(), vehicle_geom,
      /*length_buffer=*/0.0, /*width_buffer=*/kLatExtendBuffer);
  for (int j = 0, obs_num = fen_detections.detections_size(); j < obs_num;
       ++j) {
    if (fen_detections.detections(j).filtered_by_pcn() ||
        fen_detections.detections(j).filtered_by_nms()) {
      continue;
    }

    const auto obs_box = Box2d(fen_detections.detections(j).bounding_box());
    if (av_box.HasOverlap(obs_box)) return std::nullopt;
  }

  for (int i = idx; i < traj_size; ++i) {
    if (traj[i].relative_time() - traj.front().relative_time() > check_time) {
      return std::nullopt;
    }
    const auto av_point = traj[i].path_point();
    const Box2d av_box = planner::GetAvBoxWithBuffer(
        Vec2d(av_point.x(), av_point.y()), av_point.theta(), vehicle_geom,
        /*length_buffer=*/0.0, /*width_buffer=*/kLatExtendBuffer);

    for (int j = 0, obs_num = fen_detections.detections_size(); j < obs_num;
         ++j) {
      if (fen_detections.detections(j).filtered_by_pcn() ||
          fen_detections.detections(j).filtered_by_nms()) {
        continue;
      }

      const double pred_time =
          traj[i].relative_time() - start_relative_time + shift_time;
      // Filter out all fen detections which would collide with AV during the
      // entire prediction duration (i.e. driving through us).
      constexpr double kDt = 0.05;  // s
      double pred_dur = 0.0;        // s
      absl::flat_hash_map<int, bool> has_overlap_by_fen_idx;
      while (pred_dur <= pred_time) {
        Box2d obs_box = Box2d(fen_detections.detections(j).bounding_box());
        const double pred_x_move =
            fen_detections.detections(j).velocity_x() * pred_dur;
        const double pred_y_move =
            fen_detections.detections(j).velocity_y() * pred_dur;
        obs_box.Shift(Vec2d(pred_x_move, pred_y_move));

        const bool has_overlap = av_box.HasOverlap(obs_box);
        const bool is_back_or_side_collision =
            IsBackOrSideCollision(av_box, obs_box);
        if (has_overlap && !is_back_or_side_collision &&
            !FindWithDefault(has_overlap_by_fen_idx, j, false) &&
            ((pred_dur + kDt) > pred_time)) {
          return fen_detections.detections(j);
        } else if (has_overlap && is_back_or_side_collision) {
          has_overlap_by_fen_idx[j] = true;
        }
        pred_dur += kDt;
      }
    }
  }

  return std::nullopt;
}
}  // namespace emergency_brake
}  // namespace qcraft
