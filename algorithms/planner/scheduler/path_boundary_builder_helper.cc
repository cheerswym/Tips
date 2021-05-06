#include "onboard/planner/scheduler/path_boundary_builder_helper.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"

DEFINE_bool(
    planner_enable_path_boundary_debug, false,
    "Enable debug for path boundary. Will draw infos on canvas if enabled.");

namespace qcraft::planner {
namespace {

constexpr double kObjectBuffer = 0.75;            // meters.
constexpr double kVirtualStationHalfWidth = 3.0;  // meters.
constexpr double kBackWardTimeBuffer = 1.0;       // s.
constexpr double kFrontTimeBuffer = 1.5;          // s.
constexpr double kPrePursuitBuffer = 3.0;         // s.
constexpr double kPostPursuitBuffer = 1.0;        // s.

constexpr double kMinHalfLaneWidth = 0.5 * kMinLaneWidth;
constexpr double kBorrowLaneOffset = 5.2;      // m.
constexpr double kEgoLatBuffer = 0.3;          // m.
constexpr double kLcPauseEgoLatBuffer = 0.75;  // m.

constexpr double kMaxLaneChangeCancelLatAccel = 0.5;      // m/s^2.
constexpr double kComfortLaneChangeCancelLatAccel = 0.3;  // m/s^2.
constexpr double kMaxComfortLatJerk = 1.0;                // m/s^3.
constexpr int kAvTrajPointsMinSize = 5;
constexpr double kMinKinematicTrajLonSpeed = 3.0;            // m/s.
constexpr double kMinKinematicBoundaryProtectedZone = 15.0;  // m.

constexpr double kMaxLaneChangePauseRefCenterStep = 0.5;  // m.

constexpr double kIntersectionTurningMaxHalfLaneWidth = 3.5;  // m.

std::optional<double> FindPursuitTime(absl::Span<const double> av_s_vec,
                                      absl::Span<const double> av_t_vec,
                                      absl::Span<const double> obj_s_vec,
                                      absl::Span<const double> obj_t_vec) {
  QCHECK_EQ(av_s_vec.size(), av_t_vec.size());
  QCHECK_EQ(obj_s_vec.size(), obj_t_vec.size());

  for (int i = 0; i < av_t_vec.size() - 1; ++i) {
    for (int j = 0; j < obj_t_vec.size() - 1; ++j) {
      // No time overlap, ignore.
      if (av_t_vec[i + 1] < obj_t_vec[j] || av_t_vec[i] > obj_t_vec[j + 1]) {
        continue;
      }
      const Segment2d av_s_t(Vec2d(av_t_vec[i], av_s_vec[i]),
                             Vec2d(av_t_vec[i + 1], av_s_vec[i + 1]));
      Vec2d intersect_pt;
      const bool has_intersect = av_s_t.GetIntersect(
          Segment2d(Vec2d(obj_t_vec[j], obj_s_vec[j]),
                    Vec2d(obj_t_vec[j + 1], obj_s_vec[j + 1])),
          &intersect_pt);
      if (has_intersect) {
        return intersect_pt.x();
      }
    }
  }
  return std::nullopt;
}

absl::StatusOr<PiecewiseLinearFunction<double, double>>
GenerateAvTrajAlongRefCenterLine(
    const DrivePassage &drive_passage,
    const ApolloTrajectoryPointProto &plan_start_point,
    absl::Span<const Vec2d> center_xy_vec) {
  ASSIGN_OR_RETURN(const auto center_frame,
                   BuildBruteForceFrenetFrame(center_xy_vec));
  const auto cur_sl = center_frame.XYToSL(
      Vec2dFromApolloTrajectoryPointProto(plan_start_point));

  std::vector<double> vec_t, vec_s;
  vec_t.reserve(kTrajectorySteps);
  vec_s.reserve(kTrajectorySteps);

  double center_s = cur_sl.s;
  const double speed = plan_start_point.v();
  for (double t = 0; t <= kTrajectoryTimeHorizon; t += kTrajectoryTimeStep) {
    const auto center_xy = center_frame.SLToXY({center_s, 0.0});
    const auto dp_sl = drive_passage.QueryFrenetCoordinateAt(center_xy);
    if (!dp_sl.ok()) break;

    vec_t.push_back(t);
    vec_s.push_back(dp_sl->s);
    center_s += kTrajectoryTimeStep * speed;
  }

  if (vec_s.size() > 1) {
    return PiecewiseLinearFunction(vec_s, vec_t);
  } else {
    return absl::InternalError("");
  }
}

absl::StatusOr<PiecewiseLinearFunction<double, double>>
GenerateConstLateralAccelConstSpeedTraj(
    const DrivePassage &drive_passage,
    const ApolloTrajectoryPointProto &plan_start_point,
    const FrenetCoordinate &cur_sl, double target_lane_offset,
    double max_lane_change_lat_accel) {
  ASSIGN_OR_RETURN(const auto lane_tangent,
                   drive_passage.QueryTangentAtS(cur_sl.s),
                   _ << "Unable to find lane tangent at current s.");

  std::vector<double> vec_l, vec_s;
  vec_l.reserve(kTrajectorySteps);
  vec_s.reserve(kTrajectorySteps);

  constexpr double kReachTargetLaneThreshold = 0.1;  // m.
  constexpr double kNearTargetLaneThreshold = 0.3;   // m.
  constexpr double kZeroLateralVelThreshold = 0.01;  // m/s.
  constexpr double dt = kTrajectoryTimeStep;
  const auto heading_tangent =
      Vec2d::FastUnitFromAngle(plan_start_point.path_point().theta());
  double s, l, lat_v, lat_a, speed;
  s = cur_sl.s;
  l = cur_sl.l;
  speed = plan_start_point.v();
  lat_v = speed * lane_tangent.CrossProd(heading_tangent);
  lat_a = plan_start_point.a() * lane_tangent.CrossProd(heading_tangent) +
          speed * lane_tangent.Dot(heading_tangent) * speed *
              plan_start_point.path_point().kappa();
  lat_v = std::fabs(lat_v) < kZeroLateralVelThreshold ? 0.0 : lat_v;
  speed = std::max(speed, kMinKinematicTrajLonSpeed);
  vec_l.push_back(l);
  vec_s.push_back(s);

  const auto lat_accel_func = [target_lane_offset, max_lane_change_lat_accel](
                                  double l, double lat_v) {
    // Choose different accelerations for whether heading to the target lane.
    const double l_diff = l - target_lane_offset;
    double max_lat_accel;
    if (std::fabs(l_diff) < kNearTargetLaneThreshold) {
      max_lat_accel = kMaxLaneChangeCancelLatAccel;
    } else {
      max_lat_accel = lat_v * l_diff < 0.0 ? max_lane_change_lat_accel
                                           : kMaxLaneChangeCancelLatAccel;
    }
    return -std::copysign(max_lat_accel, l_diff);
  };
  const double expect_lat_a = lat_accel_func(l, lat_v);

  // Assume const lateral jerk until lateral acceleration reaches expected const
  // accel value.
  const int const_lat_jerk_steps =
      FloorToInt(std::abs(expect_lat_a - lat_a) / kMaxComfortLatJerk / dt);
  for (int i = 0; i < const_lat_jerk_steps; ++i) {
    l += lat_v * dt;
    s += speed * dt;
    lat_v += lat_a * dt;
    lat_a += std::copysign(kMaxComfortLatJerk, expect_lat_a - lat_a) * dt;
    vec_l.push_back(l);
    vec_s.push_back(s);
  }
  // Assume const lateral acceleration for the rest of trajectory.
  lat_a = lat_accel_func(l, lat_v);
  for (int i = 0; i < kTrajectorySteps - const_lat_jerk_steps - 1; ++i) {
    l += lat_v * dt;
    s += speed * dt;
    lat_v += lat_a * dt;

    if ((std::fabs(l - target_lane_offset) < kReachTargetLaneThreshold &&
         std::fabs(lat_v) < kZeroLateralVelThreshold) ||
        (l - target_lane_offset) * (vec_l.back() - target_lane_offset) < 0.0) {
      break;
    }
    vec_l.push_back(l);
    vec_s.push_back(s);
  }

  if (vec_s.size() < kAvTrajPointsMinSize) {
    return absl::NotFoundError("");
  }

  const int extend_traj_size = CeilToInt(vec_s.size() * 0.1);
  for (int i = 0; i < extend_traj_size; ++i) {
    if (vec_s.size() >= kTrajectorySteps) break;
    vec_l.push_back(target_lane_offset);
    vec_s.push_back(s);
    s += speed * dt;
  }
  return PiecewiseLinearFunction(vec_s, vec_l);
}

PathBoundary BuildBoundaryForStationaryObject(const DrivePassage &drive_passage,
                                              absl::Span<const double> center_l,
                                              const FrenetBox &obj_fbox,
                                              absl::Span<const double> s_vec,
                                              PathBoundary boundary) {
  const int n = drive_passage.stations().size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  const auto index =
      std::lower_bound(s_vec.begin(), s_vec.end(), obj_fbox.s_max) -
      s_vec.begin();
  auto index_near = index;
  if (index_near > n - 1) {
    index_near = n - 1;
  }
  auto index_far = index_near;
  while (index_near > 0 && s_vec[index_near] >= obj_fbox.s_min) {
    --index_near;
  }
  while (index_far < n - 1 && s_vec[index_far] <= obj_fbox.s_max) {
    ++index_far;
  }

  if (obj_fbox.l_max < center_l[index]) {  // Right side.
    const double lat_offset = obj_fbox.l_max;
    for (int i = index_near; i <= index_far; ++i) {
      boundary.OuterClampRightByIndex(i, lat_offset + kObjectBuffer);
    }
  } else if (obj_fbox.l_min > center_l[index]) {  // Left side.
    const double lat_offset = obj_fbox.l_min;
    for (int i = index_near; i <= index_far; ++i) {
      boundary.OuterClampLeftByIndex(i, lat_offset - kObjectBuffer);
    }
  }

  return boundary;
}

PathBoundary BuildBoundaryForDynamicObject(
    const DrivePassage &drive_passage, absl::Span<const double> center_l,
    const SpacetimeObjectTrajectory &traj,
    const PiecewiseLinearFunction<double, double> &av_t_s,
    absl::Span<const double> s_vec, PathBoundary boundary) {
  const auto &av_s_vec = av_t_s.x();
  const auto &av_t_vec = av_t_s.y();
  const int num_stations = s_vec.size();
  const int time_num_steps = traj.states().size();

  absl::flat_hash_map<int, FrenetBox> frenet_box_map;
  std::vector<double> obj_s_vec, obj_t_vec, obj_max_l_vec, obj_min_l_vec;
  obj_s_vec.reserve(time_num_steps);
  obj_t_vec.reserve(time_num_steps);
  obj_max_l_vec.reserve(time_num_steps);
  obj_min_l_vec.reserve(time_num_steps);

  for (int i = 0; i < time_num_steps; ++i) {
    ASSIGN_OR_CONTINUE(auto fbox,
                       drive_passage.QueryFrenetBoxAt(traj.states()[i].box));

    const auto index =
        std::lower_bound(s_vec.begin(), s_vec.end(), fbox.s_min) -
        s_vec.begin();
    if (index == s_vec.size()) continue;

    // If dynamic object on the target lane or cross the target lane, ignore.
    if (fbox.l_min < center_l[index] && fbox.l_max > center_l[index]) {
      continue;
    }
    obj_s_vec.push_back(0.5 * (fbox.s_max + fbox.s_min));
    obj_t_vec.push_back(i * kTrajectoryTimeStep);
    obj_max_l_vec.push_back(fbox.l_max);
    obj_min_l_vec.push_back(fbox.l_min);

    frenet_box_map[i] = std::move(fbox);
  }

  // 1. Calculate a point where the s of AV and object meets (the time which
  // the obj catches up with the AV), at this point AV should not interfer with
  // the object's path (with a time buffer).
  if (av_t_vec.size() > 1 && obj_t_vec.size() > 1) {
    const auto pursuit_time =
        FindPursuitTime(av_s_vec, av_t_vec, obj_s_vec, obj_t_vec);
    if (pursuit_time.has_value()) {
      const PiecewiseLinearFunction obj_max_l_t(obj_t_vec, obj_max_l_vec);
      const PiecewiseLinearFunction obj_min_l_t(obj_t_vec, obj_min_l_vec);
      const auto pursuit_max_l = obj_max_l_t(*pursuit_time);
      const auto pursuit_min_l = obj_min_l_t(*pursuit_time);
      const int pursuit_idx =
          static_cast<int>(*pursuit_time / kTrajectoryTimeStep);
      const int pursuit_min_idx = std::max(
          0, pursuit_idx -
                 static_cast<int>(kPrePursuitBuffer / kTrajectoryTimeStep));
      const int pursuit_max_idx = std::min(
          time_num_steps - 1,
          pursuit_idx +
              static_cast<int>(kPostPursuitBuffer / kTrajectoryTimeStep));
      for (int i = 0; i < num_stations; ++i) {
        const auto s = s_vec[i];
        if (s < av_s_vec.front() || s > av_s_vec.back()) {
          continue;
        }
        const int av_arrive_t =
            static_cast<int>(av_t_s(s) / kTrajectoryTimeStep);
        if (av_arrive_t < 0 || av_arrive_t >= time_num_steps ||
            av_arrive_t < pursuit_min_idx || av_arrive_t > pursuit_max_idx) {
          continue;
        }
        if (pursuit_max_l < center_l[i]) {
          boundary.OuterClampRightByIndex(i, pursuit_max_l + kObjectBuffer);
        }
        if (pursuit_min_l > center_l[i]) {
          boundary.OuterClampLeftByIndex(i, pursuit_min_l - kObjectBuffer);
        }
      }
    }
  }

  // 2. Consider a time buffer zone around the object, AV should not interfer
  // with the object's path within the buffer zone.
  std::vector<double> vec_min_l, vec_max_l, vec_min_s, vec_max_s;
  vec_min_l.reserve(time_num_steps);
  vec_max_l.reserve(time_num_steps);
  vec_min_s.reserve(time_num_steps);
  vec_max_s.reserve(time_num_steps);
  const int backward_buffer =
      static_cast<int>(kBackWardTimeBuffer / kTrajectoryTimeStep);
  const int front_buffer =
      static_cast<int>(kFrontTimeBuffer / kTrajectoryTimeStep);
  for (int i = 0; i < time_num_steps; ++i) {
    double min_l = std::numeric_limits<double>::max();
    double max_l = std::numeric_limits<double>::lowest();
    double min_s = std::numeric_limits<double>::max();
    double max_s = std::numeric_limits<double>::lowest();
    bool has_value = false;
    for (int j = std::max(0, i - backward_buffer);
         j < std::min(time_num_steps, i + front_buffer + 1); ++j) {
      const auto fbox_ptr = FindOrNull(frenet_box_map, j);
      if (fbox_ptr) {
        has_value = true;
        min_l = std::min(min_l, fbox_ptr->l_min);
        max_l = std::max(max_l, fbox_ptr->l_max);
        min_s = std::min(min_s, fbox_ptr->s_min);
        max_s = std::max(max_s, fbox_ptr->s_max);
      }
    }
    if (has_value) {
      vec_min_l.push_back(min_l);
      vec_max_l.push_back(max_l);
      vec_min_s.push_back(min_s);
      vec_max_s.push_back(max_s);
    } else {
      vec_min_l.push_back(std::numeric_limits<double>::lowest());
      vec_max_l.push_back(std::numeric_limits<double>::lowest());
      vec_min_s.push_back(std::numeric_limits<double>::lowest());
      vec_max_s.push_back(std::numeric_limits<double>::lowest());
    }
  }

  for (int i = 0; i < num_stations; ++i) {
    const auto s = s_vec[i];
    if (s < av_s_vec.front() || s > av_s_vec.back()) {
      continue;
    }
    const int av_arrive_t = static_cast<int>(av_t_s(s) / kTrajectoryTimeStep);
    if (av_arrive_t < 0 || av_arrive_t >= time_num_steps) {
      continue;
    }
    // Region match.
    if (s > vec_min_s[av_arrive_t] && s <= vec_max_s[av_arrive_t]) {
      if (vec_max_l[av_arrive_t] < center_l[i]) {
        boundary.OuterClampRightByIndex(i,
                                        vec_max_l[av_arrive_t] + kObjectBuffer);
      }
      if (vec_min_l[av_arrive_t] > center_l[i]) {
        boundary.OuterClampLeftByIndex(i,
                                       vec_min_l[av_arrive_t] - kObjectBuffer);
      }
    }
  }

  return boundary;
}

}  // namespace

bool IsTurningLanePath(const PlannerSemanticMapManager &psmm,
                       mapping::ElementId lane_id) {
  const auto &lane_info = psmm.FindLaneInfoOrDie(lane_id);
  return lane_info.is_in_intersection &&
         (lane_info.direction == mapping::LaneProto::LEFT_TURN ||
          lane_info.direction == mapping::LaneProto::RIGHT_TURN);
}

PathBoundary BuildPathBoundaryFromTargetLane(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    bool borrow_lane_boundary) {
  const int n = drive_passage.size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  for (const auto &station : drive_passage.stations()) {
    auto &left_bound = left_bound_vec.emplace_back();
    auto &right_bound = right_bound_vec.emplace_back();

    if (borrow_lane_boundary) {
      right_bound = -kBorrowLaneOffset;
      left_bound = kBorrowLaneOffset;
      continue;
    }

    if (station.direction() != mapping::LaneProto::STRAIGHT) {
      const auto *lane_info =
          psmm.FindLaneInfoOrNull(station.GetLanePoint().lane_id());
      right_bound = -kVirtualStationHalfWidth;
      left_bound = kVirtualStationHalfWidth;
      if (station.direction() == mapping::LaneProto::UTURN) {
        // NOTE(zixuan): Assume all uturns turn to left.
        right_bound = -kDefaultHalfLaneWidth;
        continue;
      }
      if (lane_info != nullptr) {
        if (station.direction() == mapping::LaneProto::LEFT_TURN) {
          right_bound = -kDefaultHalfLaneWidth;
          left_bound = lane_info->lane_neighbors_on_left.size() > 0
                           ? left_bound
                           : kIntersectionTurningMaxHalfLaneWidth;
        } else if (station.direction() == mapping::LaneProto::RIGHT_TURN) {
          right_bound = lane_info->lane_neighbors_on_right.size() > 0
                            ? right_bound
                            : -kIntersectionTurningMaxHalfLaneWidth;
          left_bound = kDefaultHalfLaneWidth;
        }
      }
      continue;
    }

    double right_l = -std::numeric_limits<double>::infinity();
    double left_l = std::numeric_limits<double>::infinity();
    for (const auto &bound : station.boundaries()) {
      if (-kMaxHalfLaneWidth < bound.lat_offset &&
          bound.lat_offset < -kMinHalfLaneWidth) {
        if (bound.lat_offset > right_l) {
          right_l = bound.lat_offset;
        }
      }

      if (kMinHalfLaneWidth < bound.lat_offset &&
          bound.lat_offset < kMaxHalfLaneWidth) {
        if (bound.lat_offset < left_l) {
          left_l = bound.lat_offset;
        }
      }
    }

    right_bound = right_l == -std::numeric_limits<double>::infinity()
                      ? -kDefaultHalfLaneWidth
                      : right_l;

    left_bound = left_l == std::numeric_limits<double>::infinity()
                     ? kDefaultHalfLaneWidth
                     : left_l;
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary BuildCurbPathBoundary(const DrivePassage &drive_passage) {
  const int n = drive_passage.stations().size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  for (const auto &station : drive_passage.stations()) {
    const auto [right_curb, left_curb] = station.QueryCurbOffsetAt(0.0).value();
    right_bound_vec.push_back(right_curb);
    left_bound_vec.push_back(left_curb);
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary BuildSolidPathBoundary(
    const DrivePassage &drive_passage, const FrenetCoordinate &cur_sl,
    const VehicleGeometryParamsProto &vehicle_geom,
    const ApolloTrajectoryPointProto &plan_start_point,
    double target_lane_offset) {
  const int n = drive_passage.size();
  std::vector<double> left_bound_vec, right_bound_vec;
  left_bound_vec.reserve(n);
  right_bound_vec.reserve(n);

  const double right_max_l = target_lane_offset - vehicle_geom.width() * 0.5;
  const double left_min_l = target_lane_offset + vehicle_geom.width() * 0.5;
  for (int i = 0; i < n; ++i) {
    const auto &station = drive_passage.station(StationIndex(i));
    const auto [right_curb, left_curb] = station.QueryCurbOffsetAt(0.0).value();
    double right_l = right_curb;
    double left_l = left_curb;

    for (const auto &bound : station.boundaries()) {
      if (bound.lat_offset <= right_max_l && bound.lat_offset > right_l &&
          bound.IsSolid()) {
        right_l = bound.lat_offset;
      }

      if (bound.lat_offset >= left_min_l && bound.lat_offset < left_l &&
          bound.IsSolid()) {
        left_l = bound.lat_offset;
      }
    }
    right_bound_vec.push_back(right_l);
    left_bound_vec.push_back(left_l);
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary BuildPathBoundaryFromAvKinematics(
    const DrivePassage &drive_passage,
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geom,
    const FrenetCoordinate &cur_sl, const FrenetBox &sl_box,
    absl::Span<const double> s_vec, double target_lane_offset,
    double max_lane_change_lat_accel, bool lane_change_pause) {
  const auto traj_l_s = GenerateConstLateralAccelConstSpeedTraj(
      drive_passage, plan_start_point, cur_sl, target_lane_offset,
      max_lane_change_lat_accel);
  if (traj_l_s.ok() && FLAGS_planner_enable_path_boundary_debug) {
    SendAvSlTrajectoryToCanvas(
        drive_passage, *traj_l_s,
        /*topic=*/absl::StrCat("av_sl_traj_", max_lane_change_lat_accel));
  }

  const int n = drive_passage.size();
  const double half_av_width = vehicle_geom.width() * 0.5;
  const double ego_lat_buffer = half_av_width + kEgoLatBuffer;
  std::vector<double> right_bound_vec, left_bound_vec;
  right_bound_vec.reserve(n);
  left_bound_vec.reserve(n);

  for (int i = 0; i < drive_passage.size(); ++i) {
    const auto s = s_vec[i];
    auto &right_bound = right_bound_vec.emplace_back();
    auto &left_bound = left_bound_vec.emplace_back();

    right_bound = std::numeric_limits<double>::infinity();
    left_bound = -std::numeric_limits<double>::infinity();
    if (traj_l_s.ok()) {
      if (s >= traj_l_s->x().front() && s <= traj_l_s->x().back()) {
        right_bound = traj_l_s->Evaluate(s) - ego_lat_buffer;
        left_bound = traj_l_s->Evaluate(s) + ego_lat_buffer;
      }
    }
    // TODO(zixuan): maybe useless.
    if (s < sl_box.s_max + kMinKinematicBoundaryProtectedZone) {
      right_bound = std::min(right_bound, sl_box.l_min - kEgoLatBuffer);
      left_bound = std::max(left_bound, sl_box.l_max + kEgoLatBuffer);
    }
    if (lane_change_pause) {
      right_bound = std::min(right_bound, target_lane_offset - half_av_width);
      left_bound = std::max(left_bound, target_lane_offset + half_av_width);
    }
  }

  return PathBoundary(std::move(right_bound_vec), std::move(left_bound_vec));
}

PathBoundary ShrinkPathBoundaryForLaneChangePause(
    const VehicleGeometryParamsProto &vehicle_geom, const FrenetBox &sl_box,
    const LaneChangeStateProto &lc_state, PathBoundary boundary,
    double target_lane_offset) {
  const double half_av_width = vehicle_geom.width() * 0.5;
  if (lc_state.lc_left()) {
    double shrinked_l =
        std::max(target_lane_offset + half_av_width, sl_box.l_max) +
        kLcPauseEgoLatBuffer;
    for (int i = 0; i < boundary.size(); ++i) {
      boundary.OuterClampLeftByIndex(i, shrinked_l);
    }
  } else {
    double shrinked_l =
        std::min(target_lane_offset - half_av_width, sl_box.l_min) -
        kLcPauseEgoLatBuffer;
    for (int i = 0; i < boundary.size(); ++i) {
      boundary.OuterClampRightByIndex(i, shrinked_l);
    }
  }

  return boundary;
}

PathBoundary ShrinkPathBoundaryForObject(
    const DrivePassage &drive_passage,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const ApolloTrajectoryPointProto &plan_start_point,
    absl::Span<const double> s_vec, absl::Span<const double> center_l,
    absl::Span<const Vec2d> center_xy, PathBoundary boundary) {
  for (const auto &traj : st_traj_mgr.stationary_object_trajs()) {
    const auto obj_fbox_or =
        drive_passage.QueryFrenetBoxAtContour(traj->states().front().contour);
    if (!obj_fbox_or.ok()) {
      continue;
    }
    const auto &obj_fbox = obj_fbox_or.value();
    boundary = BuildBoundaryForStationaryObject(
        drive_passage, center_l, obj_fbox, s_vec, std::move(boundary));
  }
  const auto av_t_s = GenerateAvTrajAlongRefCenterLine(
      drive_passage, plan_start_point, center_xy);
  if (av_t_s.ok()) {
    for (const auto &traj : st_traj_mgr.moving_object_trajs()) {
      boundary = BuildBoundaryForDynamicObject(
          drive_passage, center_l, *traj, *av_t_s, s_vec, std::move(boundary));
    }
  }

  return boundary;
}

double ComputeTargetLaneOffset(
    const DrivePassage &drive_passage, const FrenetCoordinate &cur_sl,
    const LaneChangeStateProto &lc_state,
    const ApolloTrajectoryPointProto plan_start_point, double half_av_width) {
  double target_lane_offset = 0.0;
  if (lc_state.stage() != LaneChangeStage::LCS_PAUSE) {
    return target_lane_offset;
  }

  double kinematic_lat_offset = cur_sl.l;
  const auto lane_tangent = drive_passage.QueryTangentAtS(cur_sl.s);
  if (lane_tangent.ok()) {
    const double lat_v =
        plan_start_point.v() * lane_tangent->CrossProd(Vec2d::FastUnitFromAngle(
                                   plan_start_point.path_point().theta()));
    kinematic_lat_offset += std::copysign(
        Sqr(lat_v) * 0.5 / kComfortLaneChangeCancelLatAccel, lat_v);
  }

  constexpr double kMinLateralBuffer = 0.1;  // m.
  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(cur_sl.s);
  // To deal with virtual lanes with no boundaries other than curbs.
  const double right_offset =
      std::max(boundaries.first->lat_offset, -kMaxHalfLaneWidth);
  const double left_offset =
      std::min(boundaries.second->lat_offset, kMaxHalfLaneWidth);
  if (lc_state.lc_left()) {
    target_lane_offset = std::max(
        right_offset - half_av_width - kMinLateralBuffer, kinematic_lat_offset);
  } else {
    target_lane_offset = std::min(
        left_offset + half_av_width + kMinLateralBuffer, kinematic_lat_offset);
  }

  if (std::abs(cur_sl.l - target_lane_offset) >
      kMaxLaneChangePauseRefCenterStep) {
    target_lane_offset =
        cur_sl.l + std::copysign(kMaxLaneChangePauseRefCenterStep,
                                 target_lane_offset - cur_sl.l);
  }

  return target_lane_offset;
}

PathSlBoundary BuildPathSlBoundary(const DrivePassage &drive_passage,
                                   std::vector<double> s_vec,
                                   std::vector<double> ref_center_l,
                                   PathBoundary inner_boundary,
                                   PathBoundary outer_boundary) {
  const int n = drive_passage.size();
  std::vector<Vec2d> inner_right_xy, inner_left_xy, outer_right_xy,
      outer_left_xy, ref_center_xy;
  inner_right_xy.reserve(n);
  inner_left_xy.reserve(n);
  outer_right_xy.reserve(n);
  outer_left_xy.reserve(n);
  for (int i = 0; i < n; ++i) {
    const auto &station = drive_passage.station(StationIndex(i));
    inner_left_xy.emplace_back(station.lat_point(inner_boundary.left(i)));
    inner_right_xy.emplace_back(station.lat_point(inner_boundary.right(i)));
    outer_right_xy.emplace_back(station.lat_point(outer_boundary.right(i)));
    outer_left_xy.emplace_back(station.lat_point(outer_boundary.left(i)));
    ref_center_xy.push_back(station.lat_point(ref_center_l[i]));
  }

  return PathSlBoundary(std::move(s_vec), std::move(ref_center_l),
                        std::move(outer_boundary.right_vec()),
                        std::move(outer_boundary.left_vec()),
                        std::move(inner_boundary.right_vec()),
                        std::move(inner_boundary.left_vec()),
                        std::move(ref_center_xy), std::move(outer_right_xy),
                        std::move(outer_left_xy), std::move(inner_right_xy),
                        std::move(inner_left_xy));
}

std::vector<double> ComputeSmoothedReferenceLine(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    const SmoothedReferenceLineResultMap &smooth_result_map) {
  const int n = drive_passage.size();
  std::vector<double> smoothed_reference_center(n, 0.0);
  absl::flat_hash_map<qcraft::mapping::ElementId,
                      PiecewiseLinearFunction<double, double>>
      lane_id_to_smoothed_lateral_offset;

  int cur_begin = -1;
  const auto &lane_path = drive_passage.lane_path();
  for (int i = 0; i < lane_path.size(); ++i) {
    const bool should_smooth = IsTurningLanePath(psmm, lane_path.lane_id(i));

    if (cur_begin == -1 && should_smooth) {
      // rise.
      cur_begin = i;
    }
    if (cur_begin != -1 && !should_smooth) {
      // drop.
      const std::vector<mapping::ElementId> lane_ids(
          lane_path.lane_ids().begin() + cur_begin,
          lane_path.lane_ids().begin() + i);
      const auto smoothed_result =
          smooth_result_map.FindOverlapSmoothedResult(lane_ids);
      if (smoothed_result.ok()) {
        lane_id_to_smoothed_lateral_offset.insert(
            smoothed_result->lane_id_to_smoothed_lateral_offset.begin(),
            smoothed_result->lane_id_to_smoothed_lateral_offset.end());
      }
      cur_begin = -1;
    }
  }
  if (cur_begin != -1) {
    // drop.
    const std::vector<mapping::ElementId> lane_ids(
        lane_path.lane_ids().begin() + cur_begin, lane_path.lane_ids().end());
    const auto smoothed_result =
        smooth_result_map.FindOverlapSmoothedResult(lane_ids);
    if (smoothed_result.ok()) {
      lane_id_to_smoothed_lateral_offset.insert(
          smoothed_result->lane_id_to_smoothed_lateral_offset.begin(),
          smoothed_result->lane_id_to_smoothed_lateral_offset.end());
    }
  }

  SmoothedReferenceCenterResult smooth_results = {
      .lane_id_to_smoothed_lateral_offset =
          std::move(lane_id_to_smoothed_lateral_offset)};
  for (int i = 0; i < n; ++i) {
    const mapping::LanePoint &lane_point =
        drive_passage.station(StationIndex(i)).GetLanePoint();
    const auto smoothed_l = smooth_results.GetSmoothedLateralOffset(lane_point);
    if (smoothed_l.ok()) {
      smoothed_reference_center[i] = *smoothed_l;
    }
  }

  return smoothed_reference_center;
}

}  // namespace qcraft::planner
