
#include "onboard/prediction/util/trajectory_developer.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "absl/status/statusor.h"
#include "onboard/math/line_fitter.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kLenEpsilon = 1e-1;
constexpr int kMaxPolyFitDegree = 5;
constexpr double kTimeEpsilon = 1e-3;
constexpr double kPurePursuitSearchStep = 3.0;
constexpr double kMinPurePursuitPathSpeed = 3.0;
constexpr double kMinLookAheadDistance = 3.0;
constexpr double kMaxLookAheadDistance = 40.0;
constexpr double kStartWeight = 100.0;
const PiecewiseLinearFunction<double, double> kPurePursuitSpeedLimitPlf(
    std::vector<double>{5.0, 10.0, 15.0, 30.0},
    std::vector<double>{3.0, 5.0, 8.0, 15.0});

// Some lateral speed related params.
constexpr int kLateralSpeedFitMinPts = 3;
constexpr double kLateralSpeedFitMinTime = 0.5;
constexpr int kSpeedFitMinPts = 3;

PiecewiseLinearFunction<double, double> IntersectioExitOffsetRatioPLF(
    {0.0, 5.0, 20.0}, {1.0, 0.7, 0.0});
double ComputePurePursuitCurvatureFromTargetPoint(const Vec2d& start,
                                                  double heading,
                                                  const Vec2d& end) {
  const auto target = end - start;
  const double angle_diff = target.FastAngle() - heading;
  return (2 * sin(angle_diff)) / std::max(target.norm(), kLenEpsilon);
}

absl::StatusOr<double> ComputePurePursuitCurvature(
    const BicycleModelState& cur_state, const planner::DrivePassage& dp,
    double min_ld, double max_ld, double lateral_offset, double preview_t,
    bool is_reversed = false) {
  const Vec2d cur_pos = Vec2d(cur_state.x, cur_state.y);
  ASSIGN_OR_RETURN(const auto frenet_sl,
                   dp.QueryLaterallyUnboundedFrenetCoordinateAt(cur_pos));
  double ld = cur_state.v * preview_t;
  ld = std::clamp(ld, min_ld, max_ld);
  // start line search
  constexpr double kSearchMultiplier = 2.0;
  std::optional<Vec2d> cur_pt_or = std::nullopt;
  double max_dis = 0.0;
  std::optional<Vec2d> max_pt_or = std::nullopt;
  // we start from s0 + l0 to ensure a smooth/proper convergence to the
  // reference line
  if (!is_reversed) {
    for (double s = frenet_sl.s + fabs(frenet_sl.l);
         s < frenet_sl.s + kSearchMultiplier * ld;
         s += kPurePursuitSearchStep) {
      auto target_pt_or = dp.QueryPointXYAtSL(s, lateral_offset);
      if (!target_pt_or.ok()) {
        break;
      }
      auto target_pt = std::move(target_pt_or).value();
      const double dis = (target_pt - cur_pos).norm();
      if (dis >= ld) {
        cur_pt_or = std::move(target_pt);
        break;
      }
      if (dis > max_dis) {
        max_pt_or = target_pt;
        max_dis = dis;
      }
    }
  } else {
    for (double s = frenet_sl.s - fabs(frenet_sl.l);
         s > frenet_sl.s - kSearchMultiplier * ld;
         s -= kPurePursuitSearchStep) {
      auto target_pt_or = dp.QueryPointXYAtSL(s, lateral_offset);
      if (!target_pt_or.ok()) {
        break;
      }
      auto target_pt = std::move(target_pt_or).value();
      const double dis = (target_pt - cur_pos).norm();
      if (dis >= ld) {
        cur_pt_or = std::move(target_pt);
        break;
      }
      if (dis > max_dis) {
        max_pt_or = target_pt;
        max_dis = dis;
      }
    }
  }
  // compute curvature
  // if we find the good point
  if (cur_pt_or.has_value()) {
    return ComputePurePursuitCurvatureFromTargetPoint(
        cur_pos, cur_state.heading, *cur_pt_or);
  }
  // if we fail to find the good point, we check if we have a OK point
  if (max_pt_or.has_value()) {
    if (max_dis > min_ld) {
      return ComputePurePursuitCurvatureFromTargetPoint(
          cur_pos, cur_state.heading, *max_pt_or);
    }
  }
  return absl::NotFoundError("Fail to find the target point");
}

absl::StatusOr<double> ComputePurePursuitCurvatureAtIntersection(
    const BicycleModelState& cur_state, const planner::DrivePassage& dp,
    double min_ld, double max_ld, double lateral_offset, double preview_t) {
  const Vec2d cur_pos = Vec2d(cur_state.x, cur_state.y);
  ASSIGN_OR_RETURN(const auto frenet_sl,
                   dp.QueryLaterallyUnboundedFrenetCoordinateAt(cur_pos));
  double ld = cur_state.v * preview_t;
  ld = std::clamp(ld, min_ld, max_ld);
  // start line search
  constexpr double kSearchMultiplier = 2.0;
  std::optional<Vec2d> cur_pt_or = std::nullopt;
  double max_dis = 0.0;
  std::optional<Vec2d> max_pt_or = std::nullopt;
  const auto nearest_station_idx = dp.FindNearestStationIndexAtS(frenet_sl.s);
  double junction_exit_s = 0.0;
  for (auto idx = nearest_station_idx; idx.value() < dp.size(); ++idx) {
    if (!dp.station(idx).is_in_intersection()) {
      junction_exit_s = dp.station(idx).accumulated_s();
      break;
    }
  }
  // we start from s0 + l0 to ensure a smooth/proper convergence to the
  // reference line
  for (double s = frenet_sl.s + fabs(frenet_sl.l);
       s < frenet_sl.s + kSearchMultiplier * ld; s += kPurePursuitSearchStep) {
    auto target_pt_or = dp.QueryPointXYAtSL(
        s, lateral_offset * IntersectioExitOffsetRatioPLF(s - junction_exit_s));
    if (!target_pt_or.ok()) {
      break;
    }
    auto target_pt = std::move(target_pt_or).value();
    const double dis = (target_pt - cur_pos).norm();
    if (dis >= ld) {
      cur_pt_or = std::move(target_pt);
      break;
    }
    if (dis > max_dis) {
      max_pt_or = target_pt;
      max_dis = dis;
    }
  }

  // compute curvature
  // if we find the good point
  if (cur_pt_or.has_value()) {
    return ComputePurePursuitCurvatureFromTargetPoint(
        cur_pos, cur_state.heading, *cur_pt_or);
  }
  // if we fail to find the good point, we check if we have a OK point
  if (max_pt_or.has_value()) {
    if (max_dis > min_ld) {
      return ComputePurePursuitCurvatureFromTargetPoint(
          cur_pos, cur_state.heading, *max_pt_or);
    }
  }
  return absl::NotFoundError("Fail to find the target point");
}

}  // namespace

std::vector<PredictedTrajectoryPoint> DevelopStaticTrajectory(
    const UniCycleState& state, double dt, double horizon) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);

  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(0.0);
  init_pt.set_a(0.0);
  points.push_back(init_pt);
  for (int i = 1; i < num_pts; ++i) {
    PredictedTrajectoryPoint next_pt = init_pt;
    next_pt.set_t(dt * i);
    points.push_back(std::move(next_pt));
  }
  return points;
}

std::vector<PredictedTrajectoryPoint> DevelopCTRATrajectory(
    const UniCycleState& state, double dt, double stop_time, double horizon,
    bool use_acc) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);
  const int stop_pts = static_cast<int>(stop_time / dt) + 1;
  auto cur_state = state;
  if (!use_acc) {
    cur_state.acc = 0.0;
  }
  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(cur_state.x, cur_state.y));
  init_pt.set_theta(cur_state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(cur_state.v);
  init_pt.set_a(cur_state.acc);
  points.push_back(init_pt);

  for (int i = 1; i < num_pts; ++i) {
    if (i >= stop_pts) {
      cur_state.yaw_rate = 0.0;
      cur_state.acc = 0.0;
    }
    const auto next_state = SimulateUniCycleModel(cur_state, dt);
    PredictedTrajectoryPoint next_pt;
    const auto& prev_pt = points.back();
    next_pt.set_t(i * dt);
    next_pt.set_pos(Vec2d(next_state.x, next_state.y));
    next_pt.set_s(prev_pt.s() + (next_pt.pos() - prev_pt.pos()).norm());
    next_pt.set_theta(next_state.heading);
    next_pt.set_v(next_state.v);
    next_pt.set_kappa(0.0);
    next_pt.set_a(next_state.acc);
    cur_state = next_state;
    points.push_back(std::move(next_pt));
  }
  return points;
}

std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const PredictionObject& obj, double dt, double horizon, bool is_reversed) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);

  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(obj.pos().x(), obj.pos().y()));
  init_pt.set_theta(obj.heading());
  init_pt.set_kappa(0.0);
  init_pt.set_v(obj.v());
  init_pt.set_a(0.0);
  const Vec2d heading_normal = Vec2d::FastUnitFromAngle(init_pt.theta());
  points.push_back(std::move(init_pt));
  for (int i = 1; i < num_pts; ++i) {
    PredictedTrajectoryPoint next_pt;
    next_pt.set_t(dt * i);
    next_pt.set_s(init_pt.v() * dt * i);
    if (is_reversed) {
      next_pt.set_pos(init_pt.pos() - heading_normal * obj.v() * dt * i);
    } else {
      next_pt.set_pos(init_pt.pos() + heading_normal * obj.v() * dt * i);
    }
    next_pt.set_theta(init_pt.theta());
    next_pt.set_kappa(0.0);
    next_pt.set_v(init_pt.v());
    next_pt.set_a(0.0);
    points.push_back(std::move(next_pt));
  }
  return points;
}
std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const UniCycleState& state, double dt, double horizon, bool is_reversed) {
  std::vector<PredictedTrajectoryPoint> points;
  const int num_pts = static_cast<int>(horizon / dt);

  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(state.v);
  init_pt.set_a(0.0);
  const Vec2d heading_normal = Vec2d::FastUnitFromAngle(init_pt.theta());
  points.push_back(std::move(init_pt));
  for (int i = 1; i < num_pts; ++i) {
    PredictedTrajectoryPoint next_pt;
    next_pt.set_t(dt * i);
    next_pt.set_s(init_pt.v() * dt * i);
    if (is_reversed) {
      next_pt.set_pos(init_pt.pos() - heading_normal * state.v * dt * i);
    } else {
      next_pt.set_pos(init_pt.pos() + heading_normal * state.v * dt * i);
    }
    next_pt.set_theta(init_pt.theta());
    next_pt.set_kappa(0.0);
    next_pt.set_v(init_pt.v());
    next_pt.set_a(0.0);
    points.push_back(std::move(next_pt));
  }
  return points;
}

std::vector<PredictedTrajectoryPoint>
CombinePathAndSpeedForPredictedTrajectoryPoints(
    const planner::DiscretizedPath& path,
    const planner::SpeedVector& speed_data) {
  std::vector<PredictedTrajectoryPoint> trajectory;
  for (int i = 0; i < speed_data.size(); ++i) {
    VLOG(3) << "speed_data:[" << i << "] = " << speed_data[i].DebugString();
  }
  if (path.size() <= 1) {
    return trajectory;
  }
  QCHECK_GT(speed_data.size(), 1);

  trajectory.clear();
  double t = 0.0;
  while (t < speed_data.TotalTime()) {
    const auto speed_point = speed_data.EvaluateByTime(t);
    if (!speed_point.has_value()) {
      return trajectory;
    }

    PathPoint path_point;
    if (path.length() < kLenEpsilon) {
      path_point = path.front();
    } else if (path.length() < speed_point->s()) {
      return trajectory;
    } else {
      path_point = path.Evaluate(speed_point->s());
    }
    PredictedTrajectoryPoint traj_point;
    traj_point.set_t(t);
    traj_point.set_s(speed_point->s());
    traj_point.set_pos(Vec2d(path_point.x(), path_point.y()));
    traj_point.set_theta(path_point.theta());
    traj_point.set_kappa(path_point.kappa());
    traj_point.set_v(speed_point->v());
    traj_point.set_a(speed_point->a());
    trajectory.push_back(std::move(traj_point));
    t += kPredictionTimeStep;
  }
  return trajectory;
}

planner::DiscretizedPath PredictedTrajectoryPointsToDiscretizedPath(
    absl::Span<const PredictedTrajectoryPoint> points) {
  planner::DiscretizedPath path;
  path.reserve(points.size());
  for (const auto& pt : points) {
    PathPoint path_pt;
    path_pt.set_x(pt.pos().x());
    path_pt.set_y(pt.pos().y());
    path_pt.set_s(pt.s());
    path_pt.set_theta(pt.theta());
    path_pt.set_kappa(pt.kappa());
    path.push_back(std::move(path_pt));
  }
  return path;
}

planner::DiscretizedPath DevelopPurePursuitPathAtIntersection(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    double lane_offset, double preview_t, double dt, double horizon, double lf,
    double lr, double max_curvature) {
  planner::DiscretizedPath path;
  path.reserve(static_cast<int>(horizon / dt));
  QCHECK_GE(state.v, 0.0);
  PathPoint init_pt;
  init_pt.set_s(0.0);
  init_pt.set_x(state.x);
  init_pt.set_y(state.y);
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);

  const double cur_v = std::fabs(state.v);
  const double nominal_v = std::clamp(cur_v, kMinPurePursuitPathSpeed,
                                      kPurePursuitSpeedLimitPlf(cur_v));
  const double path_len = cur_v * horizon;

  BicycleModelState cur_bicycle_state = state;
  cur_bicycle_state.v = nominal_v;

  const double dis_to_rac = lf - 0.5 * (lf + lr);
  // compute pseudo control input
  BicycleModelState rear_pt = cur_bicycle_state;
  rear_pt.x = cur_bicycle_state.x - cos(cur_bicycle_state.heading) * dis_to_rac;
  rear_pt.y = cur_bicycle_state.y - sin(cur_bicycle_state.heading) * dis_to_rac;

  path.push_back(std::move(init_pt));
  // forward simulation of the object's movement using purepursuit controller
  double s = 0.0;
  while (s < path_len) {
    const auto curvature_or = ComputePurePursuitCurvatureAtIntersection(
        rear_pt, dp, kMinLookAheadDistance, kMaxLookAheadDistance, lane_offset,
        preview_t);
    if (!curvature_or.ok()) {
      VLOG(2) << "curvature search failed: " << curvature_or.status();
      break;
    }
    const double wheelbase = kLengthToWheelbasePlf(lf + lr);
    const double steer_cmd = atan(*curvature_or * wheelbase);
    auto new_bicycle_state =
        UpdateBicycleStatebySteer(rear_pt, steer_cmd, lf + lr);
    const Vec2d next_pos(new_bicycle_state.x, new_bicycle_state.y);
    s += (next_pos - Vec2d(path.back().x(), path.back().y())).norm();
    PathPoint next_pt;
    const auto center_pos =
        next_pos + dis_to_rac * Vec2d::UnitFromAngle(new_bicycle_state.heading);
    next_pt.set_x(center_pos.x());
    next_pt.set_y(center_pos.y());
    next_pt.set_s(s);
    next_pt.set_theta(new_bicycle_state.heading);
    next_pt.set_kappa(std::tan(new_bicycle_state.front_wheel_angle) /
                      wheelbase);
    path.push_back(std::move(next_pt));
    rear_pt = std::move(new_bicycle_state);
  }

  return path;
}
BicycleModelState ObjectProtoToBicycleModelState(
    const ObjectProto& object_proto) {
  BicycleModelState state;
  QCHECK(object_proto.has_id());
  const auto pos = Vec2dFromProto(object_proto.pos());
  state.x = pos.x();
  state.y = pos.y();
  state.v = Vec2dFromProto(object_proto.vel()).norm();
  state.heading = NormalizeAngle(object_proto.yaw());
  state.acc = 0.0;
  state.front_wheel_angle = 0.0;
  return state;
}

BicycleModelState UpdateBicycleStatebySteer(const BicycleModelState& state,
                                            double steer_cmd, double length) {
  // KSteerInertia is for imitating steering delay.
  const double wheelbase = kLengthToWheelbasePlf(length);
  const double max_steer = kLengthToMaxFrontSteerPlf(length);
  const double KSteerInertia = kPredictionTimeStep;
  BicycleModelState update_bicycle_state = state;

  // Steering wheel model
  update_bicycle_state.front_wheel_angle =
      (steer_cmd + state.front_wheel_angle * KSteerInertia) /
      (KSteerInertia + 1.0);

  // Assume max steer rate is from 0 to max steer in 1s.
  update_bicycle_state.front_wheel_angle =
      std::clamp(update_bicycle_state.front_wheel_angle,
                 state.front_wheel_angle - max_steer * kPredictionTimeStep,
                 state.front_wheel_angle + max_steer * kPredictionTimeStep);

  update_bicycle_state.front_wheel_angle =
      std::clamp(update_bicycle_state.front_wheel_angle, -max_steer, max_steer);

  update_bicycle_state.v = state.v + state.acc * kPredictionTimeStep;

  if (update_bicycle_state.v * state.v <= 0.0) {
    // Prevent vehicle speed change direction.
    update_bicycle_state.v = 0.0;
  } else {
    update_bicycle_state.x = state.x + update_bicycle_state.v *
                                           fast_math::Cos(state.heading) *
                                           kPredictionTimeStep;
    update_bicycle_state.y = state.y + update_bicycle_state.v *
                                           fast_math::Sin(state.heading) *
                                           kPredictionTimeStep;
    update_bicycle_state.heading = NormalizeAngle(
        state.heading + update_bicycle_state.v / wheelbase *
                            std::tan(update_bicycle_state.front_wheel_angle) *
                            kPredictionTimeStep);
  }
  return update_bicycle_state;
}

std::vector<PredictedTrajectoryPoint> DevelopConstVelocityPurePursuitTrajectory(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    double lane_offset, double preview_t, double min_look_ahead,
    double max_look_ahead, double dt, double horizon, double lf, double lr,
    double max_curvature, bool is_reverse_driving) {
  const int num_pts = static_cast<int>(horizon / dt);
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(num_pts);
  QCHECK_GE(state.v, 0.0);
  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(std::fabs(state.v));
  init_pt.set_a(0.0);
  // obstacle info to bicycle state
  BicycleModelState cur_bicycle_state = state;
  cur_bicycle_state.acc = 0.0;

  const double dis_to_rac = 0.5 * (lf + lr) - lr;
  // compute pseudo control input
  BicycleModelState rear_pt = cur_bicycle_state;
  rear_pt.x = cur_bicycle_state.x - cos(cur_bicycle_state.heading) * dis_to_rac;
  rear_pt.y = cur_bicycle_state.y - sin(cur_bicycle_state.heading) * dis_to_rac;

  points.push_back(std::move(init_pt));
  // forward simulation of the object's movement using purepursuit controller
  double s = 0.0;
  for (int i = 1; i < num_pts; ++i) {
    const auto curvature_or =
        ComputePurePursuitCurvature(rear_pt, dp, min_look_ahead, max_look_ahead,
                                    lane_offset, preview_t, is_reverse_driving);
    if (!curvature_or.ok()) {
      break;
    }

    const double wheelbase = kLengthToWheelbasePlf(lf + lr);
    const double steer_cmd = atan(*curvature_or * wheelbase);
    auto new_bicycle_state =
        UpdateBicycleStatebySteer(rear_pt, steer_cmd, lf + lr);
    const Vec2d next_pos(new_bicycle_state.x, new_bicycle_state.y);
    s += (next_pos - points.back().pos()).norm();
    PredictedTrajectoryPoint next_pt;
    next_pt.set_t(dt * i);
    next_pt.set_pos(next_pos + dis_to_rac * Vec2d::UnitFromAngle(
                                                new_bicycle_state.heading));
    next_pt.set_s(s);
    next_pt.set_theta(new_bicycle_state.heading);
    next_pt.set_kappa(std::tan(new_bicycle_state.front_wheel_angle) /
                      wheelbase);
    next_pt.set_v(points.back().v());
    next_pt.set_a(0.0);
    points.push_back(std::move(next_pt));
    rear_pt = std::move(new_bicycle_state);
  }

  return points;
}
planner::DiscretizedPath DevelopPurePursuitPath(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    double lane_offset, double preview_t, double dt, double horizon, double lf,
    double lr, double max_curvature) {
  planner::DiscretizedPath path;
  path.reserve(static_cast<int>(horizon / dt));
  PathPoint init_pt;
  init_pt.set_s(0.0);
  init_pt.set_x(state.x);
  init_pt.set_y(state.y);
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  const double cur_v = state.v;
  const double nominal_v = std::clamp(cur_v, kMinPurePursuitPathSpeed,
                                      kPurePursuitSpeedLimitPlf(cur_v));
  const double path_len = std::max(cur_v, nominal_v) * horizon;
  // Use a nominal bicycle state v
  BicycleModelState cur_bicycle_state = state;
  cur_bicycle_state.v = nominal_v;
  const double dis_to_rac = lf - 0.5 * (lf + lr);
  // compute pseudo control input
  BicycleModelState rear_pt = cur_bicycle_state;
  rear_pt.x = cur_bicycle_state.x - cos(cur_bicycle_state.heading) * dis_to_rac;
  rear_pt.y = cur_bicycle_state.y - sin(cur_bicycle_state.heading) * dis_to_rac;
  path.push_back(std::move(init_pt));
  // forward simulation of the object's movement using purepursuit controller
  double s = 0.0;
  while (s < path_len) {
    const auto curvature_or = ComputePurePursuitCurvature(
        rear_pt, dp, kMinLookAheadDistance, kMaxLookAheadDistance, lane_offset,
        preview_t);
    if (!curvature_or.ok()) {
      VLOG(2) << "curvature search failed: " << curvature_or.status();
      break;
    }
    const double wheelbase = kLengthToWheelbasePlf(lf + lr);
    const double steer_cmd = atan(*curvature_or * wheelbase);
    auto new_bicycle_state =
        UpdateBicycleStatebySteer(rear_pt, steer_cmd, lf + lr);
    const Vec2d next_pos(new_bicycle_state.x, new_bicycle_state.y);
    s += (next_pos - Vec2d(path.back().x(), path.back().y())).norm();
    PathPoint next_pt;
    const auto center_pos =
        next_pos + dis_to_rac * Vec2d::UnitFromAngle(new_bicycle_state.heading);
    next_pt.set_x(center_pos.x());
    next_pt.set_y(center_pos.y());
    next_pt.set_s(s);
    next_pt.set_theta(new_bicycle_state.heading);
    next_pt.set_kappa(std::tan(new_bicycle_state.front_wheel_angle) /
                      wheelbase);
    path.push_back(std::move(next_pt));
    rear_pt = std::move(new_bicycle_state);
  }
  return path;
}

planner::SpeedVector DevelopConstVelocitySpeedProfile(double cur_speed,
                                                      double dt,
                                                      double horizon) {
  // Apply acceleration
  double t = 0.0;
  planner::SpeedVector speed_vec;
  const double dt_inverse = 1.0 / dt;
  speed_vec.reserve(static_cast<int>(horizon * dt_inverse));
  planner::SpeedPoint sp0(t, /*double s=*/0.0, cur_speed, /*accel=*/0.0,
                          /*double jerk=*/0.0);
  speed_vec.push_back(std::move(sp0));
  t += dt;
  while (t < horizon) {
    const auto& prev = speed_vec.back();
    planner::SpeedPoint sp1;
    sp1.set_t(t);
    sp1.set_v(cur_speed);
    sp1.set_s(prev.s() + 0.5 * (prev.v() + cur_speed) * dt);
    sp1.set_a((cur_speed - prev.v()) * dt_inverse);
    speed_vec.push_back(std::move(sp1));

    t += dt;
  }
  return speed_vec;
}
absl::StatusOr<std::vector<PredictedTrajectoryPoint>>
DevelopConstVelocityPolePlacementTrajectory(const BicycleModelState& state,
                                            const planner::DrivePassage& dp,
                                            double dt, double horizon,
                                            double obj_len) {
  constexpr double kDs = 1.0;  // Sample interval of drive passage.
  constexpr double kReferencePointDs = 0.2;  // m.
  std::vector<PredictedTrajectoryPoint> res;
  const auto frenet_or = dp.QueryFrenetCoordinateAt(Vec2d(state.x, state.y));
  if (!frenet_or.ok()) {
    return res;
  }
  const double nominal_dv = 5.0;
  const double sample_dist = horizon * (state.v + nominal_dv);
  const int target_num = static_cast<int>(sample_dist / kDs) + 1;
  std::vector<Vec2d> x_s, y_s;
  std::vector<double> weights;
  x_s.reserve(target_num);
  y_s.reserve(target_num);
  weights.reserve(target_num);
  double last_s = 0.0;
  for (int idx = 0; idx < target_num && kDs * idx <= sample_dist; ++idx) {
    const double s = kDs * idx;
    auto pt_or = dp.QueryPointXYAtS(frenet_or->s + s);
    if (!pt_or.ok()) {
      break;
    }
    x_s.push_back(Vec2d(s, pt_or->x()));
    y_s.push_back(Vec2d(s, pt_or->y()));
    if (idx == 0) {
      weights.push_back(kStartWeight);
    } else {
      weights.push_back(1.0);
    }
    last_s = s;
  }
  PolynomialFitter x_fitter, y_fitter;
  const int degree =
      std::min(static_cast<int>(x_s.size()) - 1, kMaxPolyFitDegree);
  if (degree < 1) {
    return absl::NotFoundError("Reference path not long enough");
  }
  x_fitter.SetDegree(degree);
  y_fitter.SetDegree(degree);
  x_fitter.LoadData(x_s, weights);
  y_fitter.LoadData(y_s, weights);
  x_fitter.FitData(LS_SOLVER::QR);
  y_fitter.FitData(LS_SOLVER::QR);

  std::vector<Vec2d> reference_points;
  std::vector<double> ks;
  const int ref_num =
      static_cast<int>(std::floor(last_s / kReferencePointDs)) + 1;
  reference_points.reserve(ref_num);
  ks.reserve(ref_num);
  for (int i = 0; i < ref_num; ++i) {
    const double t = i * kReferencePointDs;
    const double x = x_fitter.Evaluate(t);
    const double dx = x_fitter.EvaluateDerivative(t, 1);
    const double d2x = x_fitter.EvaluateDerivative(t, 2);
    const double y = y_fitter.Evaluate(t);
    const double dy = y_fitter.EvaluateDerivative(t, 1);
    const double d2y = y_fitter.EvaluateDerivative(t, 2);
    reference_points.push_back(Vec2d(x, y));
    ks.push_back((dx * d2y - dy * d2x) / std::pow(dx * dx + dy * dy, 1.5));
  }
  return res;
}

PredictedTrajectory BuildPredictedTrajectory(
    absl::Span<const PredictedTrajectoryPoint> traj_pts,
    absl::string_view annotation, double probability,
    const ObjectPredictionPriority priority, PredictionType pred_type) {
  PredictedTrajectory traj;
  traj.set_annotation(std::string(annotation));
  traj.set_type(pred_type);
  traj.set_probability(probability);
  traj.set_priority(priority);
  *traj.mutable_points() =
      std::vector<PredictedTrajectoryPoint>(traj_pts.begin(), traj_pts.end());
  return traj;
}

double LineFitLateralSpeedByHistory(const ObjectHistorySpan& history,
                                    const planner::DrivePassage& dp) {
  if (history.size() <= 1) return 0.0;
  const double origin_ts = history.back().time;
  std::vector<Vec2d> data_points;
  data_points.reserve(history.size());
  std::vector<double> weights;
  weights.reserve(history.size());
  for (const auto& node : history) {
    const auto obj_pos = node.val.pos();
    const auto frenet_sl_or = dp.QueryFrenetCoordinateAt(obj_pos);
    if (!frenet_sl_or.ok()) {
      continue;
    }
    const double time_diff_in_second = node.time - origin_ts;
    data_points.emplace_back(Vec2d(time_diff_in_second, frenet_sl_or->l));
    weights.emplace_back(1.0);  // use same weight for now.
  }
  if (data_points.size() < kLateralSpeedFitMinPts ||
      data_points.front().x() > -kLateralSpeedFitMinTime) {
    return 0.0;
  }
  LineFitter fitter;
  fitter.LoadData(data_points, weights);
  fitter.FitData();
  const auto tangent = fitter.tangent();
  if (std::fabs(tangent.x()) < kTimeEpsilon) {
    return 0.0;
  }
  return tangent.y() / tangent.x();
}
double LineFitAccelerationByHistory(const ObjectHistorySpan& history) {
  if (history.size() <= 1) return 0.0;
  const double origin_ts = history.back().time;
  std::vector<Vec2d> data_points;
  data_points.reserve(history.size());
  std::vector<double> weights;
  weights.reserve(history.size());
  for (const auto& node : history) {
    const auto v = node.val.v();
    const double time_diff_in_second = node.time - origin_ts;
    data_points.emplace_back(Vec2d(time_diff_in_second, v));
    weights.emplace_back(1.0);  // use same weight for now.
  }
  if (data_points.size() < kSpeedFitMinPts) {
    return 0.0;
  }
  LineFitter fitter;
  fitter.LoadData(data_points, weights);
  fitter.FitData();
  const auto tangent = fitter.tangent();
  if (std::fabs(tangent.x()) < kTimeEpsilon) {
    return 0.0;
  }
  return tangent.y() / tangent.x();
}

}  // namespace prediction
}  // namespace qcraft
