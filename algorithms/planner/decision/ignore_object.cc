#include "onboard/planner/decision/ignore_object.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "onboard/eval/qevent.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {
namespace planner {
namespace {
struct State {
  Vec2d pos;
  double theta;  // Heading angle.
  double delta;  // Steering angle.
};

std::vector<State> GenerateKinematicPath(
    const qcraft::VehicleGeometryParamsProto& vehicle_gem,
    const Vec2d av_pos_smooth, const Vec2d av_v, double av_theta, double phi,
    double accel, double delta_t) {
  double v = av_v.norm();
  // This value calculated by robot-bus Q8001 (max_steer_angle / steer_ratio).
  // Strictly, this value should be calculated by params in
  // `VehicleDriveParamsProto`, because this params is not fixed for different
  // vehicles. But here use this const value is ok due to AV maybe never reach
  // max steer angle. Meanwhile, this param is almost same for different
  // vehicles.
  constexpr double kMaxDelta = 0.57;
  constexpr int kStatesNum = 50;

  std::vector<State> states;
  states.reserve(kStatesNum);
  // Add init state.
  states.push_back({.pos = av_pos_smooth, .theta = av_theta, .delta = 0.0});

  for (int i = 0; i < kStatesNum; ++i) {
    // Kinematic bicycle model.
    const Vec2d xy_dot = v * Vec2d::FastUnitFromAngle(states[i].theta);
    const double theta_dot = v * std::tan(states[i].delta) /
                             vehicle_gem.wheel_base();  // rotation rate (𝜔)
    const double delta_dot = phi;

    v = v + accel * delta_t;
    if (v <= 0.0) {
      break;
    }

    // Derivation next state.
    State next_state;
    next_state.pos = states[i].pos + xy_dot * delta_t;
    next_state.theta = states[i].theta + theta_dot * delta_t;
    next_state.delta = states[i].delta + delta_dot * delta_t;
    if (next_state.delta > kMaxDelta) {
      next_state.delta = kMaxDelta;
    }
    states.push_back(std::move(next_state));
  }
  return states;
}

// This function construct swept volume polygon by sample points of given path.
std::optional<Polygon2d> ComputeSweptVolumePolygon(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const Vec2d av_pos_smooth, const Vec2d av_v, double av_theta, double phi,
    double control_lat_accuracy) {
  constexpr double kVehicleMaxDecel = -5.0;  // m/s^2
  constexpr double kDt = 0.1;                // s
  const std::vector<State> states = GenerateKinematicPath(
      vehicle_geom, av_pos_smooth, av_v, av_theta, phi, kVehicleMaxDecel, kDt);
  if (states.size() < 2) {
    return std::nullopt;
  }
  int sample_number = 7;
  if (states.size() < sample_number) {
    sample_number = 1;
  }
  const int sample_step = static_cast<int>(states.size() / sample_number);
  const double av_half_width =
      vehicle_geom.width() * 0.5 + control_lat_accuracy;

  // Construct polygon.
  std::vector<Vec2d> vertices;
  vertices.reserve((sample_number + 2) * 2);
  int i = 0;
  for (; i < states.size(); i += sample_step) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(states[i].theta);
    const Vec2d front_center =
        states[i].pos + tangent * vehicle_geom.front_edge_to_center();
    vertices.push_back(front_center + tangent.Perp() * av_half_width);
    vertices.push_back(front_center - tangent.Perp() * av_half_width);
  }
  // Add last state vertices.
  if (i != states.size() - 1) {
    const Vec2d tangent = Vec2d::FastUnitFromAngle(states.back().theta);
    const Vec2d front_center =
        states.back().pos + tangent * vehicle_geom.front_edge_to_center();
    vertices.push_back(front_center + tangent.Perp() * av_half_width);
    vertices.push_back(front_center - tangent.Perp() * av_half_width);
  }

  Polygon2d swept_volume_polygon;
  Polygon2d::ComputeConvexHull(vertices, &swept_volume_polygon);
  return swept_volume_polygon;
}

// Returns whether AV can turn right or turn left, the first value is turn right
// flag, the second value is turn left flag.
std::optional<std::pair<bool, bool>> EstimateDrivingBehavior(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const Vec2d av_pos_smooth) {
  const auto& nearest_station = passage.FindNearestStation(av_pos_smooth);
  const auto curbs = passage.QueryCurbOffsetAt(nearest_station.xy());
  if (!curbs.ok()) {
    return std::nullopt;
  }

  // If av not inside drive passage due to control error, set 0.0 as default.
  double av_lat_offset = 0.0;
  if (auto av_lat_offset_or = passage.QueryFrenetLatOffsetAt(av_pos_smooth);
      av_lat_offset_or.ok()) {
    av_lat_offset = *av_lat_offset_or;
  }

  const double half_vehicle_width = vehicle_geom.width() * 0.5;
  // Variant `right_gap` and `left_gap` is always a positive number if AV inside
  // drive passage.
  const double right_gap =
      std::max(av_lat_offset - curbs->first - half_vehicle_width, 0.0);
  const double left_gap =
      std::max(curbs->second - av_lat_offset - half_vehicle_width, 0.0);
  constexpr double kAvPassableBuffer = 1.5;  // m

  // Check whether AV can turn right.
  bool is_turn_right_available = false;
  if (right_gap > vehicle_geom.width() + kAvPassableBuffer) {
    is_turn_right_available = true;
  }
  // Check whether AV can turn left.
  bool is_turn_left_available = false;
  if (left_gap > vehicle_geom.width() + kAvPassableBuffer) {
    is_turn_left_available = true;
  }
  return std::make_pair(is_turn_right_available, is_turn_left_available);
}

// This function compute unavoidable collision area based on av driving
// behavior.
std::optional<Polygon2d> ComputeUnavoidableCollisionArea(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const Vec2d av_pos_smooth, const Vec2d av_v,
    double av_theta, double control_lat_accuracy) {
  const auto is_turn_right_left_available =
      EstimateDrivingBehavior(vehicle_geom, passage, av_pos_smooth);
  if (!is_turn_right_left_available.has_value()) {
    return std::nullopt;
  }

  // Steering angle rate (phi).
  constexpr double kGoStraightPhi = 0.0;
  constexpr double kTurnRightPhi = -0.3;
  constexpr double kTurnLeftPhi = 0.3;
  // Compute unavoidable collision area based on driving behavior.
  Polygon2d unavoidable_collision_area;
  if (!is_turn_right_left_available->first ||
      !is_turn_right_left_available->second) {
    const auto braking_to_stop_polygon =
        ComputeSweptVolumePolygon(vehicle_geom, av_pos_smooth, av_v, av_theta,
                                  kGoStraightPhi, control_lat_accuracy);
    if (!braking_to_stop_polygon.has_value()) {
      return std::nullopt;
    }
    unavoidable_collision_area = braking_to_stop_polygon.value();
  }

  if (is_turn_right_left_available->first &&
      is_turn_right_left_available->second) {
    const auto turn_right_polygon =
        ComputeSweptVolumePolygon(vehicle_geom, av_pos_smooth, av_v, av_theta,
                                  kTurnRightPhi, control_lat_accuracy);
    const auto turn_left_polygon =
        ComputeSweptVolumePolygon(vehicle_geom, av_pos_smooth, av_v, av_theta,
                                  kTurnLeftPhi, control_lat_accuracy);
    if (!turn_right_polygon.has_value() || !turn_left_polygon.has_value()) {
      return std::nullopt;
    }
    turn_right_polygon->ComputeOverlap(turn_left_polygon.value(),
                                       &unavoidable_collision_area);
  } else if (is_turn_right_left_available->first) {
    const auto turn_right_polygon =
        ComputeSweptVolumePolygon(vehicle_geom, av_pos_smooth, av_v, av_theta,
                                  kTurnRightPhi, control_lat_accuracy);
    if (!turn_right_polygon.has_value()) {
      return std::nullopt;
    }
    unavoidable_collision_area.ComputeOverlap(turn_right_polygon.value(),
                                              &unavoidable_collision_area);
  } else if (is_turn_right_left_available->second) {
    const auto turn_left_polygon =
        ComputeSweptVolumePolygon(vehicle_geom, av_pos_smooth, av_v, av_theta,
                                  kTurnLeftPhi, control_lat_accuracy);
    if (!turn_left_polygon.has_value()) {
      return std::nullopt;
    }
    unavoidable_collision_area.ComputeOverlap(turn_left_polygon.value(),
                                              &unavoidable_collision_area);
  }

  return unavoidable_collision_area;
}

std::optional<ConstraintProto::IgnoreObjectProto>
FodOrVegetationAvCannotStopBefore(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const SpacetimeObjectTrajectory& st_obj,
    const Vec2d av_pos_smooth, const Vec2d av_v, double av_theta,
    double control_lat_accuracy) {
  if (st_obj.planner_object()->type() == OT_FOD ||
      st_obj.planner_object()->type() == OT_VEGETATION) {
    VLOG(3) << "Object: " << st_obj.traj_id()
            << " type is FOD or VEGETATION, considering ignore.";

    const auto unavoidable_collision_area =
        ComputeUnavoidableCollisionArea(vehicle_geom, passage, av_pos_smooth,
                                        av_v, av_theta, control_lat_accuracy);
    if (!unavoidable_collision_area.has_value()) {
      return std::nullopt;
    }

    if (unavoidable_collision_area.value().HasOverlap(
            st_obj.planner_object()->contour())) {
      ConstraintProto::IgnoreObjectProto ignore_object;
      ignore_object.set_traj_id(std::string(st_obj.traj_id()));
      if (st_obj.planner_object()->type() == OT_FOD) {
        ignore_object.set_reason(
            ConstraintProto::IgnoreObjectProto::CANNOT_STOP_BEFORE_FOD);
      } else if (st_obj.planner_object()->type() == OT_VEGETATION) {
        ignore_object.set_reason(
            ConstraintProto::IgnoreObjectProto::CANNOT_STOP_BEFORE_VEGETATION);
      }
      return ignore_object;
    }
  }
  return std::nullopt;
}
}  // namespace

std::vector<ConstraintProto::IgnoreObjectProto> FindObjectsToIgnore(
    const qcraft::VehicleGeometryParamsProto& vehicle_geom,
    const DrivePassage& passage, const SpacetimeTrajectoryManager& st_traj_mgr,
    const PoseProto& av_pose) {
  const Vec2d av_pos_smooth =
      Vec2d(av_pose.pos_smooth().x(), av_pose.pos_smooth().y());
  const Vec2d av_v = Vec2d(av_pose.vel_smooth().x(), av_pose.vel_smooth().y());
  // Use control lat accuracy plf as collosion buffer. This control accuracy is
  // roughly estimated by current speed. As to decel is given as a constant
  // here.
  // Input: current lat speed (m/s). Output: lat control accuracy (m).
  const PiecewiseLinearFunction<double> control_lat_accuracy_plf({0.0, 5.0},
                                                                 {0.1, 0.2});
  const double control_lat_accuracy = control_lat_accuracy_plf(av_v.y());

  std::vector<ConstraintProto::IgnoreObjectProto> ignore_objects;
  for (const auto* st_obj : st_traj_mgr.trajectories()) {
    const auto none_stopable_object = FodOrVegetationAvCannotStopBefore(
        vehicle_geom, passage, *st_obj, av_pos_smooth, av_v, av_pose.yaw(),
        control_lat_accuracy);
    if (none_stopable_object.has_value()) {
      QEVENT_EVERY_N_SECONDS(
          "yumeng", "ignore_none_stopable_fod_or_vegetation",
          /*seconds=*/1.0, [&](QEvent* qevent) {
            qevent->AddField("object_id", none_stopable_object->traj_id())
                .AddField("object_type",
                          QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(
                                             none_stopable_object->traj_id()))
                              ->planner_object()
                              ->type())
                .AddField("reason", "Cannot stop before FOD or vegetation.");
          });
      ignore_objects.push_back(std::move(*none_stopable_object));
    }
  }
  return ignore_objects;
}

}  // namespace planner
}  // namespace qcraft
