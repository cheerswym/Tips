#include "onboard/control/trajectory_interface.h"

#include <algorithm>
#include <vector>

#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/control_flags.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kLateralSearchTimeThreshold = 0.1;  // s.

// Squared distance from the point to (x, y).
double PointDistanceSquare(const ApolloTrajectoryPointProto &point,
                           const double x, const double y) {
  const double dx = point.path_point().x() - x;
  const double dy = point.path_point().y() - y;
  return dx * dx + dy * dy;
}

double UpdateTransitionHeaderTime(double header_time_transitional,
                                  double header_time_input) {
  // At the first cycle, sync transition head time with the header time input.
  constexpr double kTimeThreshold = 0.3;  // s.
  if (header_time_transitional + kTimeThreshold < header_time_input) {
    return header_time_input;
  }

  return std::min(
      header_time_transitional + TrajectoryInterface::kTimeIncrement,
      header_time_input);
}

bool IsInFullyAutoMode(const boost::circular_buffer<bool> &auto_mode_cache) {
  if (auto_mode_cache.empty()) return false;
  bool in_fully_auto_mode = true;
  for (const auto auto_mode : auto_mode_cache) {
    in_fully_auto_mode = in_fully_auto_mode && auto_mode;
  }
  return in_fully_auto_mode;
}

bool ApproximatelyEqual(double a, double b) {
  return std::fabs(a - b) < kEpsilon;
}

ApolloTrajectoryPointProto ExtrapolateTrajectoryPoint(
    const ApolloTrajectoryPointProto &trajectory_point, double delta_s) {
  const double kappa = trajectory_point.path_point().kappa();
  const double theta0 = trajectory_point.path_point().theta();
  const double delta_theta = kappa * delta_s;

  // Chord length of the circle.
  // Chord_length = 2 * R * sin(0.5 * delta_theta)
  //              ~= delta_s - delta_s * (delta_theta)^2 / 24.0;
  const double chord_length = delta_s - delta_s * Sqr(delta_theta) / 24.0;

  const Vec2d p0(trajectory_point.path_point().x(),
                 trajectory_point.path_point().y());
  const Vec2d p0_to_p1 =
      chord_length * Vec2d::UnitFromAngle(theta0 + 0.5 * delta_theta);
  const Vec2d p1 = p0 + p0_to_p1;

  ApolloTrajectoryPointProto p;
  p.mutable_path_point()->set_x(p1.x());
  p.mutable_path_point()->set_y(p1.y());
  p.mutable_path_point()->set_z(trajectory_point.path_point().z());
  p.mutable_path_point()->set_theta(NormalizeAngle(theta0 + delta_theta));
  p.mutable_path_point()->set_kappa(trajectory_point.path_point().kappa());
  p.mutable_path_point()->set_s(trajectory_point.path_point().s() + delta_s);
  p.set_v(trajectory_point.v());
  p.set_a(trajectory_point.a());
  p.set_relative_time(trajectory_point.relative_time());
  p.set_lat_control_tolerance(trajectory_point.lat_control_tolerance());
  p.set_lon_control_tolerance(trajectory_point.lon_control_tolerance());

  return p;
}

ApolloTrajectoryPointProto GenerateTrajectoryPointBasedOnPathS(
    absl::Span<const ApolloTrajectoryPointProto> traj_points,
    Chassis::GearPosition trajectory_gear, double s) {
  // Reverse driving condition, trajectory point.s() is negative, which needs
  // special processing.
  auto it_lower = std::lower_bound(
      traj_points.begin(), traj_points.end(), s,
      [trajectory_gear](const ApolloTrajectoryPointProto &p, double s) {
        if (trajectory_gear == Chassis::GEAR_REVERSE) {
          return p.path_point().s() > s;
        } else {
          return p.path_point().s() < s;
        }
      });
  if (it_lower == traj_points.begin() || it_lower == traj_points.end()) {
    QLOG_EVERY_N_SEC(WARNING, 5.0)
        << "Queried path point based on position is out of "
        << "planner trajectory range. ";
    if (it_lower == traj_points.end()) {
      it_lower -= 1;
    }
    const double delta_s = s - (*it_lower).path_point().s();
    VLOG(1) << "delta_s: " << delta_s;
    return ExtrapolateTrajectoryPoint(*it_lower, delta_s);
  }

  const auto &p0 = (*(it_lower - 1));
  const auto &p1 = (*it_lower);

  const double s0 = p0.path_point().s();
  const double s1 = p1.path_point().s();
  const double alpha = s0 == s1 ? 0.0 : (s - s0) / (s1 - s0);

  return planner::LerpTrajectoryPoint(p0, p1, alpha);
}

ApolloTrajectoryPointProto GenerateTrajectoryPointBasedOnRelativeTime(
    absl::Span<const ApolloTrajectoryPointProto> traj_points,
    double relative_time) {
  auto it_lower =
      std::lower_bound(traj_points.begin(), traj_points.end(), relative_time,
                       [](const ApolloTrajectoryPointProto &p, double t) {
                         return p.relative_time() < t;
                       });
  VLOG(1) << "index_diff: " << it_lower - traj_points.begin() << '\n'
          << "s: " << (*it_lower).path_point().s();
  if (it_lower == traj_points.begin() || it_lower == traj_points.end()) {
    QLOG_EVERY_N_SEC(WARNING, 5)
        << "Queried trajectory point based on relative "
           "time is out of planner trajectory range.";
    if (it_lower == traj_points.end()) {
      it_lower -= 1;
    }

    return *it_lower;
  }

  const ApolloTrajectoryPointProto &p0 = *(it_lower - 1);
  const ApolloTrajectoryPointProto &p1 = *it_lower;

  const double t0 = p0.relative_time();
  const double t1 = p1.relative_time();

  QCHECK_NE(t0, t1);
  const double alpha = (relative_time - t0) / (t1 - t0);

  return planner::LerpTrajectoryPoint(p0, p1, alpha);
}

std::vector<ApolloTrajectoryPointProto> ComputeEStopTrajectory(
    double start_time_diff, Chassis::GearPosition trajectory_gear,
    absl::Span<const ApolloTrajectoryPointProto> all_traj_points_previous) {
  // TODO(zhichao): consider emergency stop at R-gear conditions.

  std::vector<ApolloTrajectoryPointProto> stop_trajectory_point;
  stop_trajectory_point.reserve(kMaxPastPointNum + kTrajectorySteps);

  // Compute the past points;
  for (int i = 0; i < kMaxPastPointNum + 1; ++i) {
    const double past_point_relative_time =
        (i - kMaxPastPointNum) * kTrajPointInterval + start_time_diff;
    const ApolloTrajectoryPointProto past_point =
        GenerateTrajectoryPointBasedOnRelativeTime(all_traj_points_previous,
                                                   past_point_relative_time);
    stop_trajectory_point.push_back(past_point);
    stop_trajectory_point[i].set_relative_time((i - kMaxPastPointNum) *
                                               kTrajPointInterval);
  }

  // Compute stop trajectory points with a constant longitudinal jerk, and
  // the minimal acceleration of each trajectory points is
  // control_estop_acceleration from gflag setting;
  const auto based_traj_point = stop_trajectory_point.back();
  double s = based_traj_point.path_point().s();
  double v = based_traj_point.v();
  double a = based_traj_point.a();

  for (int i = 1; i < kTrajectorySteps; ++i) {
    const double a_prev = a;
    const double v_prev = v;

    a = std::max(FLAGS_control_estop_acceleration,
                 a_prev + FLAGS_control_estop_jerk * kTrajPointInterval);
    const double j = (a - a_prev) / kTrajPointInterval;
    v += 0.5 * (a + a_prev) * kTrajPointInterval;
    v = std::max(0.0, v);
    s += 0.5 * (v + v_prev) * kTrajPointInterval;

    // If the emergency trajectory point.s is larger than the original
    // trajectory, just copy the trajectory point from the original trajectory.
    const double relative_time = start_time_diff + i * kTrajPointInterval;
    auto original_traj_point = GenerateTrajectoryPointBasedOnRelativeTime(
        all_traj_points_previous, relative_time);

    if (s > original_traj_point.path_point().s()) {
      original_traj_point.set_relative_time(i * kTrajPointInterval);
      stop_trajectory_point.push_back(original_traj_point);
      continue;
    }

    auto point = GenerateTrajectoryPointBasedOnPathS(all_traj_points_previous,
                                                     trajectory_gear, s);
    point.set_v(v);
    point.set_a(a);
    point.set_j(j);
    point.set_relative_time(i * kTrajPointInterval);

    stop_trajectory_point.push_back(point);
  }

  return stop_trajectory_point;
}

int FindNearestPointIndex(
    absl::Span<const ApolloTrajectoryPointProto> all_traj_points, double x,
    double y) {
  unsigned int index_min = 0;
  double dist_min = PointDistanceSquare(all_traj_points[0], x, y);
  for (int i = 1; i < all_traj_points.size(); ++i) {
    const double dist_temp = PointDistanceSquare(all_traj_points[i], x, y);
    if (dist_temp < dist_min) {
      index_min = i;
      dist_min = dist_temp;
    }
  }
  return index_min;
}

bool JudgeIsStationaryTrajectory(
    const TrajectoryProto &planning_published_trajectory) {
  const auto &first_path_point =
      planning_published_trajectory.trajectory_point(0).path_point();
  for (const auto &point : planning_published_trajectory.trajectory_point()) {
    if (!ApproximatelyEqual(point.v(), 0.0) ||
        !ApproximatelyEqual(point.a(), 0.0) ||
        !ApproximatelyEqual(point.j(), 0.0) ||
        !ApproximatelyEqual(point.path_point().x(), first_path_point.x()) ||
        !ApproximatelyEqual(point.path_point().y(), first_path_point.y()) ||
        !ApproximatelyEqual(point.path_point().theta(),
                            first_path_point.theta()) ||
        !ApproximatelyEqual(point.path_point().kappa(),
                            first_path_point.kappa())) {
      return false;
    }
  }
  return true;
}

absl::Status CheckPublishedTrajectory(
    const TrajectoryProto &planning_published_trajectory) {
  if (planning_published_trajectory.trajectory_point_size() == 0) {
    return absl::InvalidArgumentError(
        "Trajectory check fails: empty trajectory point;");
  }

  if (planning_published_trajectory.past_points_size() == 0) {
    return absl::InvalidArgumentError(
        "Trajectory check fails: empty past point;");
  }

  if (!planning_published_trajectory.has_gear()) {
    return absl::InvalidArgumentError(
        "Trajectory check fails: no gear command;");
  }

  return absl::OkStatus();
}

}  // namespace

const std::vector<ApolloTrajectoryPointProto>
    &TrajectoryInterface::GetAllTrajPoints(bool on_transition_traj) const {
  if (all_traj_points_transition_.second.empty()) {
    QLOG_EVERY_N_SEC(ERROR, 1.0) << "Transition trajectory is empty.";
    return all_traj_points_current_.second;
  }

  return on_transition_traj ? all_traj_points_transition_.second
                            : all_traj_points_current_.second;
}

double TrajectoryInterface::GetPlannerStartTime(bool on_transition_traj) const {
  return on_transition_traj ? all_traj_points_transition_.first
                            : all_traj_points_current_.first;
}

void TrajectoryInterface::UpdateAllTrajPointsCurrent(
    double header_time, const TrajectoryProto &planning_published_trajectory) {
  all_traj_points_current_.second.clear();
  all_traj_points_current_.first = header_time;

  const int trajectory_point_size =
      planning_published_trajectory.trajectory_point_size();
  const int past_points_size = planning_published_trajectory.past_points_size();
  all_traj_points_current_.second.reserve(trajectory_point_size +
                                          past_points_size);
  VLOG(1) << "all_traj_points_size: " << all_traj_points_current_.second.size();

  // Merge trajectory points and past points into all traj points.
  for (int i = 0; i < past_points_size; ++i) {
    all_traj_points_current_.second.push_back(
        planning_published_trajectory.past_points(i));
  }

  for (int i = 0; i < trajectory_point_size; ++i) {
    all_traj_points_current_.second.push_back(
        planning_published_trajectory.trajectory_point(i));
  }

  // Recalculate s of the trajectory_points.
  double s_update = 0.0;
  all_traj_points_current_.second[0].mutable_path_point()->set_s(s_update);
  for (int i = 1; i < all_traj_points_current_.second.size(); ++i) {
    const auto &traj_point_0 = all_traj_points_current_.second[i - 1];
    const auto &traj_point_1 = all_traj_points_current_.second[i];
    const Vec2d p0(traj_point_0.path_point().x(),
                   traj_point_0.path_point().y());
    const Vec2d p1(traj_point_1.path_point().x(),
                   traj_point_1.path_point().y());

    s_update += p0.DistanceTo(p1);
    all_traj_points_current_.second[i].mutable_path_point()->set_s(s_update);
  }

  // Consider reverse driving condition, s in future trajectory point with
  // GEAR_REVERSE should be negative, otherwise positive, it doesn't matter
  // when standstill, only have a valid point with s = 0.0.
  // 1.0 for drive forward, -1.0 for drive backwards.
  const double driving_direction =
      planning_published_trajectory.gear() == Chassis::GEAR_REVERSE ? -1.0
                                                                    : 1.0;
  const double s_base =
      all_traj_points_current_.second[past_points_size].path_point().s();
  for (int i = 0; i < all_traj_points_current_.second.size(); ++i) {
    const double s = all_traj_points_current_.second[i].path_point().s();
    all_traj_points_current_.second[i].mutable_path_point()->set_s(
        (s - s_base) * driving_direction);
    VLOG(1) << "all_traj_points_current[" << i << "].s = "
            << all_traj_points_current_.second[i].mutable_path_point()->s();
  }
}

absl::Status TrajectoryInterface::Update(
    const AutonomyStateProto &autonomy_state,
    const TrajectoryProto &planning_published_trajectory,
    ControllerDebugProto *controller_debug_proto) {
  SCOPED_QTRACE("TrajectoryInterface::Update");

  RETURN_IF_ERROR(CheckPublishedTrajectory(planning_published_trajectory));

  const double header_time_input =
      planning_published_trajectory.has_trajectory_start_timestamp()
          ? planning_published_trajectory.trajectory_start_timestamp()
          : planning_published_trajectory.header().timestamp() * 1e-6;

  aeb_triggered_ = planning_published_trajectory.aeb_triggered();
  is_stationary_trajectory_ =
      JudgeIsStationaryTrajectory(planning_published_trajectory);
  trajectory_gear_ = planning_published_trajectory.gear();
  is_low_speed_freespace_ = planning_published_trajectory.low_speed_freespace();

  const bool emergency_to_stop =
      autonomy_state.autonomy_state() == AutonomyStateProto::EMERGENCY_TO_STOP;
  controller_debug_proto->set_in_emergency_stop_state(emergency_to_stop);

  if (emergency_to_stop) {
    if (all_traj_points_previous_.second.empty()) {
      UpdateAllTrajPointsCurrent(header_time_input,
                                 planning_published_trajectory);

      all_traj_points_transition_ = all_traj_points_current_;
      all_traj_points_previous_ = all_traj_points_current_;
    }

    constexpr double kStopTrajUpdatePeriod = 0.2;
    if (ToUnixDoubleSeconds(Clock::Now()) >
        all_traj_points_current_.first + kStopTrajUpdatePeriod) {
      all_traj_points_current_.first = ToUnixDoubleSeconds(Clock::Now());
      const double start_time_diff =
          all_traj_points_current_.first - all_traj_points_previous_.first;

      all_traj_points_current_.second = ComputeEStopTrajectory(
          start_time_diff, trajectory_gear_, all_traj_points_previous_.second);

      all_traj_points_transition_ = all_traj_points_current_;
      all_traj_points_previous_ = all_traj_points_current_;
    }
    // Set the step distance between two adjacent trajectory step_s_ as 0.
    step_s_ = 0.0;
    // Save trajectory point to debug proto.
    QCHECK_GT(all_traj_points_current_.second.size(), 0);

    for (int i = 0; i < kTrajectorySteps; ++i) {
      *controller_debug_proto->add_emergency_stop_trajectory() =
          all_traj_points_current_.second[i + kMaxPastPointNum];
    }

    return absl::OkStatus();
  }

  auto_mode_cache_.push_back(IS_AUTO_DRIVE(autonomy_state.autonomy_state()));

  QCHECK(!planning_published_trajectory.trajectory_point().empty())
      << "Planner trajectory point is empty.";

  // When obtaining new planner trajectory, update both all_traj_points_current_
  // and all_traj_points_previous_;
  if (std::fabs(all_traj_points_current_.first - header_time_input) >
      kEpsilon) {
    constexpr double kDefaultInterval = 0.1;  // s.
    planner_update_interval_ =
        previous_trajectory_start_timestamp_ == 0.0
            ? kDefaultInterval
            : header_time_input - previous_trajectory_start_timestamp_;
    previous_trajectory_start_timestamp_ = header_time_input;

    all_traj_points_previous_ = all_traj_points_current_;
    UpdateAllTrajPointsCurrent(header_time_input,
                               planning_published_trajectory);

    step_s_ =
        -QueryTrajectoryPointByRelativeTime(false, -planner_update_interval_)
             .path_point()
             .s();

    // Shift previous trajectory's s() to match with current trajectory's
    // based point.
    for (auto &traj_point : all_traj_points_previous_.second) {
      const double s_before = traj_point.path_point().s();
      traj_point.mutable_path_point()->set_s(s_before - step_s_);
    }
  } else {
    step_s_ = 0.0;
  }
  controller_debug_proto->mutable_transition_trajectory_debug_proto()
      ->set_same_as_planner_trajectory(true);

  // In the first cycle, the all_traj_points_previous_ will be empty.
  if (all_traj_points_previous_.second.empty()) {
    all_traj_points_previous_ = all_traj_points_current_;
    all_traj_points_transition_ = all_traj_points_current_;
    return absl::OkStatus();
  }

  // Calculate transition trajectory below.
  all_traj_points_transition_.first = UpdateTransitionHeaderTime(
      all_traj_points_transition_.first, header_time_input);

  if (std::fabs(all_traj_points_current_.first -
                all_traj_points_previous_.first) < kEpsilon) {
    all_traj_points_transition_ = all_traj_points_current_;
    QLOG_EVERY_N(ERROR, 20)
        << "Current and previous trajectory have the same planner "
           "start time. ";
    return absl::OkStatus();
  }

  if (!IsInFullyAutoMode(auto_mode_cache_)) {
    controller_debug_proto->mutable_transition_trajectory_debug_proto()
        ->set_is_in_fully_auto_mode(false);
    all_traj_points_transition_ = all_traj_points_current_;
    return absl::OkStatus();
  }
  controller_debug_proto->mutable_transition_trajectory_debug_proto()
      ->set_is_in_fully_auto_mode(true);

  double alpha =
      (all_traj_points_transition_.first - all_traj_points_previous_.first) /
      (all_traj_points_current_.first - all_traj_points_previous_.first);
  alpha = std::clamp(alpha, 0.0, 1.0);

  if (std::fabs(alpha - 1.0) < kEpsilon) {
    all_traj_points_transition_ = all_traj_points_current_;
    return absl::OkStatus();
  }

  // Fill transition trajectory points.
  controller_debug_proto->mutable_transition_trajectory_debug_proto()
      ->set_same_as_planner_trajectory(false);
  const int index_shift =
      RoundToInt(planner_update_interval_ / kTrajPointInterval);
  const int transition_traj_point_size =
      std::min(all_traj_points_current_.second.size(),
               all_traj_points_previous_.second.size()) -
      index_shift;
  all_traj_points_transition_.second.clear();
  all_traj_points_transition_.second.reserve(transition_traj_point_size);
  for (int i = 0; i + index_shift < transition_traj_point_size; ++i) {
    const auto &traj_curr = all_traj_points_current_.second[i];

    const double relative_time_traj_prev =
        planner_update_interval_ + traj_curr.relative_time();
    const auto traj_prev = GenerateTrajectoryPointBasedOnRelativeTime(
        all_traj_points_previous_.second, relative_time_traj_prev);

    auto traj_transition =
        planner::LerpTrajectoryPoint(traj_prev, traj_curr, alpha);

    // Calculate its relative time.
    const double traj_current_absolute_time =
        all_traj_points_current_.first + traj_curr.relative_time();
    traj_transition.set_relative_time(traj_current_absolute_time -
                                      all_traj_points_transition_.first);

    all_traj_points_transition_.second.push_back(traj_transition);
  }

  // Update Debug information.
  constexpr bool kRecordTransitTraj = true;
  if (kRecordTransitTraj) {
    constexpr int kRecordInterval = 30;
    int index = 0;
    for (const auto &p : GetAllTrajPoints(kRecordTransitTraj)) {
      if (++index % kRecordInterval != 1) continue;

      *controller_debug_proto->mutable_transition_trajectory_debug_proto()
           ->add_transition_traj_point() = p;
    }
    controller_debug_proto->mutable_transition_trajectory_debug_proto()
        ->set_transition_traj_start_time(
            GetPlannerStartTime(kRecordTransitTraj));
  }

  return absl::OkStatus();
}

// Refer design doc:
// https://qcraft.feishu.cn/docs/doccnSjQFKRnK3zQEXIpynlMdhf#
ApolloTrajectoryPointProto
TrajectoryInterface::QueryNearestTrajectoryPointByPositionWithSRange(
    bool on_transition_traj, double x, double y, double last_matched_point_s,
    double av_speed) const {
  SCOPED_QTRACE(
      "TrajectoryInterface::"
      "QueryNearestTrajectoryPointByPositionWithSRange");

  double closest_point_s =
      QueryNearestTrajectoryPointByPosition(on_transition_traj, x, y)
          .path_point()
          .s();
  const double search_s_base =
      last_matched_point_s + av_speed * kTimeIncrement - step_s_;
  constexpr double kMinDiffSThreshold = 0.2;  // m
  const double max_s_diff_from_last_matched_point = std::max(
      kMinDiffSThreshold, std::fabs(av_speed * kLateralSearchTimeThreshold));
  const double valid_search_s_range_min =
      search_s_base - max_s_diff_from_last_matched_point;
  const double valid_search_s_range_max =
      search_s_base + max_s_diff_from_last_matched_point;

  if (closest_point_s <= valid_search_s_range_min ||
      closest_point_s >= valid_search_s_range_max) {
    QLOG_EVERY_N_SEC(INFO, 1.0)
        << "lateral_matched_points jumps too big" << '\n'
        << "closest_point_s: " << closest_point_s << '\n'
        << "valid_search_s_range_min: " << valid_search_s_range_min << '\n'
        << "valid_search_s_range_max: " << valid_search_s_range_max;

    QEVENT_EVERY_N_SECONDS(
        "shijun", "lateral_matched_points_jumps_too_big",
        /*every_n_seconds*/ 1.0, [&](QEvent *qevent) {
          qevent->AddField("closest_point_s", closest_point_s)
              .AddField("valid_search_s_range_min", valid_search_s_range_min)
              .AddField("valid_search_s_range_max", valid_search_s_range_max);
        });

    closest_point_s = std::clamp(closest_point_s, valid_search_s_range_min,
                                 valid_search_s_range_max);
  }

  return QueryTrajectoryPointBasedOnPathS(on_transition_traj, closest_point_s);
}

ApolloTrajectoryPointProto
TrajectoryInterface::QueryNearestTrajectoryPointByPosition(
    bool on_transition_traj, double x, double y) const {
  SCOPED_QTRACE("TrajectoryInterface::QueryNearestTrajectoryPointByPosition");
  const int nearest_point_index =
      FindNearestPointIndex(GetAllTrajPoints(on_transition_traj), x, y);
  VLOG(1) << "nearest_point_index: " << nearest_point_index;
  const auto closest_path_point =
      all_traj_points_current_.second[nearest_point_index].path_point();

  const Vec2d closest_path_point_xy(closest_path_point.x(),
                                    closest_path_point.y());
  const Vec2d closest_path_point_tangent =
      Vec2d::UnitFromAngle(closest_path_point.theta());
  const double closest_path_point_s = closest_path_point.s();
  const Vec2d pos(x, y);
  const double current_s =
      closest_path_point_s +
      (pos - closest_path_point_xy).dot(closest_path_point_tangent);

  return QueryTrajectoryPointBasedOnPathS(on_transition_traj, current_s);
}

ApolloTrajectoryPointProto
TrajectoryInterface::QueryTrajectoryPointBasedOnPathS(bool on_transition_traj,
                                                      double s) const {
  return GenerateTrajectoryPointBasedOnPathS(
      GetAllTrajPoints(on_transition_traj), trajectory_gear_, s);
}

ApolloTrajectoryPointProto
TrajectoryInterface::QueryTrajectoryPointByRelativeTime(
    bool on_transition_traj, double relative_time) const {
  return GenerateTrajectoryPointBasedOnRelativeTime(
      GetAllTrajPoints(on_transition_traj), relative_time);
}

}  // namespace qcraft::control
