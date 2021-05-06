#include "onboard/planner/planner_util.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "gflags/gflags.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/trajectory_util.h"

DEFINE_bool(planner_treat_cyclists_as_vulnerable, true,
            "Treat cyclists as vulnerable road user, using larger buffers");
DEFINE_bool(
    planner_treat_motorcyclists_as_vulnerable, true,
    "Treat motorcyclists as vulnerable road user, using larger buffers");

namespace qcraft {
namespace planner {

namespace {

void UpdateToMinSpeedLimit(absl::flat_hash_map<mapping::ElementId, double> *map,
                           mapping::ElementId lane_id, double speed_limit) {
  auto pair_it = map->insert({lane_id, speed_limit});
  if (pair_it.second == false) {
    pair_it.first->second = std::min(pair_it.first->second, speed_limit);
  }
}

}  // namespace

double ComputeLongitudinalJerk(const TrajectoryPoint &traj_point) {
  return traj_point.j() - Cube(traj_point.v()) * Sqr(traj_point.kappa());
}

double ComputeLateralAcceleration(const TrajectoryPoint &traj_point) {
  return Sqr(traj_point.v()) * traj_point.kappa();
}

double ComputeLateralJerk(const TrajectoryPoint &traj_point) {
  return 3.0 * traj_point.v() * traj_point.a() * traj_point.kappa() +
         Sqr(traj_point.v()) * traj_point.psi();
}

bool IsVulnerableRoadUserType(ObjectType type) {
  return type == OT_PEDESTRIAN ||
         (type == OT_CYCLIST && FLAGS_planner_treat_cyclists_as_vulnerable) ||
         (type == OT_MOTORCYCLIST &&
          FLAGS_planner_treat_motorcyclists_as_vulnerable);
}

bool IsStaticObjectType(ObjectType type) {
  return type == OT_UNKNOWN_STATIC || type == OT_VEGETATION || type == OT_FOD ||
         type == OT_BARRIER || type == OT_CONE;
}

mapping::LanePath FindContinuousCodirectionalLaneNeighborPath(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePath &lane_path, bool left, double max_length,
    bool broken_boundary_must, bool cut_extra) {
  if (FLAGS_semantic_map_with_sections) {
    return FindCodirectionalAlignedLaneNeighborPath(
        semantic_map_manager, lane_path, left, max_length, broken_boundary_must,
        cut_extra);
  }

  const mapping::LanePoint source_start = lane_path.front();
  const auto &current_lane_info =
      semantic_map_manager.FindLaneInfoOrDie(source_start.lane_id());
  const auto &neighbors = left ? current_lane_info.lane_neighbors_on_left
                               : current_lane_info.lane_neighbors_on_right;
  const auto neighbor_it = std::find_if(
      neighbors.begin(), neighbors.end(),
      [&source_start](const mapping::LaneNeighborInfo &neighbor) {
        return InRange(source_start.fraction(), neighbor.this_start_fraction,
                       neighbor.this_end_fraction);
      });
  if (neighbor_it == neighbors.end()) {
    QLOG(ERROR) << "No valid lane neighbor found for lane path start point "
                << source_start.DebugString() << ".";
    return mapping::LanePath(&semantic_map_manager, source_start);
  }
  if (neighbor_it->opposite) {
    QLOG(ERROR)
        << "No codirectional lane neighbor found for lane path start point "
        << source_start.DebugString() << " (the lane neighbor "
        << neighbor_it->other_id << " is going in the opposite direction.";
    return mapping::LanePath(&semantic_map_manager, source_start);
  }

  // Neighbor found. Now calculate the longitudinally corresponding point on
  // this neighbor lane.
  const auto &neighbor = *neighbor_it;
  const mapping::LanePoint dest_start(
      neighbor.other_id,
      Lerp(neighbor.other_start_fraction, neighbor.other_end_fraction,
           LerpFactor(neighbor.this_start_fraction, neighbor.this_end_fraction,
                      source_start.fraction())));
  mapping::LanePath dest_lane_path(
      &semantic_map_manager, {dest_start.lane_id()}, dest_start.fraction(),
      neighbor.other_end_fraction);
  VLOG(3) << "dest_start: " << dest_start.DebugString()
          << " dest_lane_path: " << dest_lane_path.DebugString();

  // Crawl along the original lane path and expand the range of continuous lane
  // neighbors on the neighboring lane path.
  for (auto lane_it = lane_path.begin(); lane_it != lane_path.end();
       ++lane_it) {
    const auto &lane = *lane_it;
    const auto &lane_info =
        semantic_map_manager.FindLaneInfoOrDie(lane.lane_id);
    const std::vector<mapping::LaneNeighborInfo> &neighbors =
        left ? lane_info.lane_neighbors_on_left
             : lane_info.lane_neighbors_on_right;
    VLOG(3) << "source lane: " << lane.lane_id
            << " neighbors: " << neighbors.size();
    bool change = true;
    std::vector<mapping::LaneNeighborInfo> extended_neighbor;
    while (change && dest_lane_path.length() < max_length) {
      change = false;
      for (const auto &neighbor : neighbors) {
        if (neighbor.opposite) continue;

        if (mapping::IsBoundarySolid(neighbor.lane_boundary_type)) {
          continue;
        }

        const mapping::LanePoint neighbor_start(neighbor.other_id,
                                                neighbor.other_start_fraction);
        VLOG(4) << "  codirectional neighbor " << neighbor.other_id
                << " other fractions: " << neighbor.other_start_fraction << ", "
                << neighbor.other_end_fraction << " dest_lane_path.back = "
                << dest_lane_path.back().DebugString();
        if (std::find_if(extended_neighbor.begin(), extended_neighbor.end(),
                         [&neighbor](const mapping::LaneNeighborInfo &v) {
                           return neighbor.other_id == v.other_id &&
                                  neighbor.other_start_fraction ==
                                      v.other_start_fraction &&
                                  neighbor.other_end_fraction ==
                                      v.other_end_fraction;
                         }) != extended_neighbor.end()) {
          VLOG(4) << "Already exits";
          continue;
        }
        constexpr double kConsecutiveLaneNeighborMaxGap = 4.0;  // m.
        mapping::LanePath connector_lane_path(&semantic_map_manager);
        const double connector_length = DistanceBetweenNearbyLanePoints(
            semantic_map_manager, dest_lane_path.back(), neighbor_start,
            &connector_lane_path);
        if (connector_length < kConsecutiveLaneNeighborMaxGap) {
          VLOG(4) << "  connected: length = " << connector_length
                  << " connector lane path: "
                  << connector_lane_path.DebugString();
          if (dest_lane_path.back().lane_id() != neighbor_start.lane_id()) {
            QCHECK(dest_lane_path.back() == connector_lane_path.front());
            dest_lane_path = dest_lane_path.Connect(connector_lane_path);
          }
          auto new_dest_lane_path = mapping::LanePath(
              &semantic_map_manager, dest_lane_path.lane_ids(),
              dest_lane_path.start_fraction(),
              std::max(neighbor.other_end_fraction,
                       dest_lane_path.end_fraction()));
          VLOG(4) << "  expanded dest lane path: "
                  << dest_lane_path.DebugString();
          change = new_dest_lane_path != dest_lane_path;
          dest_lane_path = std::move(new_dest_lane_path);
          extended_neighbor.emplace_back(neighbor);
        }
      }
      if (dest_lane_path.length() >= max_length) break;
    }
  }
  if (cut_extra && dest_lane_path.length() > max_length) {
    dest_lane_path = dest_lane_path.BeforeArclength(max_length);
  }
  VLOG(3) << "Final dest_lane_path: " << dest_lane_path.DebugString();
  return dest_lane_path;
}

mapping::LanePath FindCodirectionalAlignedLaneNeighborPath(
    const SemanticMapManager &semantic_map_manager,
    const mapping::LanePath &lane_path, bool left, double max_length,
    bool broken_boundary_must, bool cut_extra) {
  const auto &start_lane_info =
      semantic_map_manager.FindLaneInfoOrDie(lane_path.front().lane_id());
  const auto &neighbors = left ? start_lane_info.lane_neighbors_on_left
                               : start_lane_info.lane_neighbors_on_right;
  if (neighbors.empty()) return mapping::LanePath(&semantic_map_manager);

  const mapping::LanePoint neighbor_start_point(neighbors.front().other_id,
                                                lane_path.start_fraction());

  const auto neighbor_path =
      ForwardExtendRoutePath(semantic_map_manager, !left, neighbor_start_point,
                             lane_path, max_length, broken_boundary_must);

  if (cut_extra) return neighbor_path.BeforeArclength(max_length);
  return neighbor_path;
}

double SignedDistanceBetweenNearbyLanePoints(
    const SemanticMapManager &semantic_map_manager, mapping::LanePoint from,
    mapping::LanePoint to, mapping::LanePath *lane_path) {
  const double from_length =
      semantic_map_manager.GetLaneLengthOrDie(from.lane_id());

  if (from.lane_id() == to.lane_id() && from.fraction() <= to.fraction()) {
    // From and to are on the same lane and correctly ordered. Return the
    // portion of this lane between the two lane points.
    if (lane_path != nullptr) {
      *lane_path = mapping::LanePath(&semantic_map_manager, {from.lane_id()},
                                     from.fraction(), to.fraction());
    }
    return from_length * (to.fraction() - from.fraction());
  }

  // From is after to on the same lane, or they aren't on the same lane at all.
  const mapping::LaneProto &from_lane_proto =
      semantic_map_manager.FindLaneByIdOrDie(from.lane_id());
  if (std::find(from_lane_proto.outgoing_lanes().begin(),
                from_lane_proto.outgoing_lanes().end(),
                to.lane_id()) != from_lane_proto.outgoing_lanes().end()) {
    // to is an outgoing lane of from.
    // If from and to are on the same lane, this must be a self-loop lane.
    // Return a lane path consisting of two occurrences of this lane.
    // Otherwise, it's just a regular connected pair of lanes.
    if (lane_path != nullptr) {
      *lane_path = mapping::LanePath(&semantic_map_manager,
                                     {from.lane_id(), to.lane_id()},
                                     from.fraction(), to.fraction());
    }
    const double to_length =
        semantic_map_manager.GetLaneLengthOrDie(to.lane_id());
    return from_length * (1.0 - from.fraction()) + to_length * to.fraction();
  } else {
    // There is no short lane path connecting the two lane points.
    return std::numeric_limits<double>::infinity();
  }
}

double DistanceBetweenNearbyLanePoints(
    const SemanticMapManager &semantic_map_manager, mapping::LanePoint point0,
    mapping::LanePoint point1, mapping::LanePath *lane_path) {
  const double dist_forward = SignedDistanceBetweenNearbyLanePoints(
      semantic_map_manager, point0, point1, lane_path);
  if (!std::isinf(dist_forward)) return dist_forward;
  return SignedDistanceBetweenNearbyLanePoints(semantic_map_manager, point1,
                                               point0, lane_path);
}

PlannerSemanticMapModification CreateSemanticMapModification(
    const SemanticMapManager &semantic_map_manager,
    const mapping::SemanticMapModifierProto &modifier) {
  absl::flat_hash_map<mapping::ElementId, double> lane_speed_limit_map;
  double max_speed_limit = std::numeric_limits<double>::max();

  if (modifier.has_speed_limit_modifier()) {
    if (modifier.speed_limit_modifier().has_max_speed_limit()) {
      max_speed_limit = modifier.speed_limit_modifier().max_speed_limit();
    }

    for (const auto &lane_id_mod :
         modifier.speed_limit_modifier().lane_id_modifier()) {
      UpdateToMinSpeedLimit(&lane_speed_limit_map, lane_id_mod.lane_id(),
                            lane_id_mod.override_speed_limit());
    }

    for (const auto &region_mod :
         modifier.speed_limit_modifier().region_modifier()) {
      const Polygon2d polygon = SmoothPolygon2dFromGeoPolygonProto(
          region_mod.region(), semantic_map_manager.coordinate_converter());
      for (const auto &mutable_lane_info : semantic_map_manager.lane_info()) {
        bool in_polygon = true;
        for (const auto &sp : mutable_lane_info.points_smooth) {
          if (!polygon.IsPointIn(sp)) {
            in_polygon = false;
            break;
          }
        }
        if (in_polygon) {
          UpdateToMinSpeedLimit(&lane_speed_limit_map, mutable_lane_info.id,
                                region_mod.override_speed_limit());
        }
      }
    }
  }

  return PlannerSemanticMapModification{
      .lane_speed_limit_map = std::move(lane_speed_limit_map),
      .max_speed_limit = max_speed_limit};
}

mapping::SemanticMapModifierProto PlannerSemanticMapModificationToProto(
    const PlannerSemanticMapModification &modifier) {
  mapping::SemanticMapModifierProto modifier_proto;
  modifier_proto.mutable_speed_limit_modifier()->set_max_speed_limit(
      modifier.max_speed_limit);

  for (const auto &it : modifier.lane_speed_limit_map) {
    auto *lane_id_modifier =
        modifier_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
    lane_id_modifier->set_lane_id(it.first);
    lane_id_modifier->set_override_speed_limit(it.second);
  }

  return modifier_proto;
}

std::vector<ApolloTrajectoryPointProto> CreatePastPointsList(
    absl::Time plan_time, const ApolloTrajectoryPointProto &plan_start_point,
    const TrajectoryProto &prev_traj, bool reset) {
  std::vector<ApolloTrajectoryPointProto> past_points;
  past_points.reserve(kMaxPastPointNum);
  const double curr_plan_time = ToUnixDoubleSeconds(plan_time);
  const double prev_traj_start_time = prev_traj.trajectory_start_timestamp();
  if (prev_traj.trajectory_point().empty() ||
      curr_plan_time >
          prev_traj_start_time +
              prev_traj.trajectory_point().rbegin()->relative_time() ||
      curr_plan_time < prev_traj_start_time || reset) {
    // No valid history or history too old or history not long enough. Create a
    // constant curvature, constant acceleration kinematic history.
    VLOG(3) << "No previous trajectory or resetting. Create past "
               "points from "
               "current curvature and acceleration.";
    Vec2d plan_start_vec2d(plan_start_point.path_point().x(),
                           plan_start_point.path_point().y());

    Vec2d x = plan_start_vec2d;
    double t = 0.0;
    double s = 0.0;
    double theta = plan_start_point.path_point().theta();
    double v = plan_start_point.v();

    const double kappa = plan_start_point.path_point().kappa();
    const double a = plan_start_point.a();

    for (int i = 0; i < kMaxPastPointNum; ++i) {
      t += -kTrajectoryTimeStep;
      const double dv = -a * kTrajectoryTimeStep;
      const double ds =
          -v * kTrajectoryTimeStep + 0.5 * a * Sqr(kTrajectoryTimeStep);
      const double dtheta = kappa * ds;
      x += ds * Vec2d::FastUnitFromAngle(theta + dtheta * 0.5);
      s += ds;
      theta += dtheta;
      v = std::max(0.0, v + dv);
      // TODO(renjie): Make the past trajectory self-consistent.
      ApolloTrajectoryPointProto past_point;
      past_point.mutable_path_point()->set_x(x.x());
      past_point.mutable_path_point()->set_y(x.y());
      past_point.mutable_path_point()->set_s(s);
      past_point.mutable_path_point()->set_theta(theta);
      past_point.mutable_path_point()->set_kappa(kappa);
      past_point.mutable_path_point()->set_lambda(0.0);
      past_point.set_v(v);
      past_point.set_a(a);
      past_point.set_j(0.0);
      past_point.set_relative_time(t);
      past_points.push_back(past_point);
      VLOG(4) << "Past point at t = " << t << ": "
              << past_point.ShortDebugString();
    }
    std::reverse(past_points.begin(), past_points.end());
  } else {
    QCHECK_EQ(prev_traj.past_points_size(), kMaxPastPointNum);
    const int relative_time_index = RoundToInt(
        (curr_plan_time - prev_traj_start_time) / kTrajectoryTimeStep);
    QCHECK_GE(relative_time_index, 0);
    QCHECK_LT(relative_time_index, prev_traj.trajectory_point_size());
    for (int i = kMaxPastPointNum; i > 0; --i) {
      const int index = relative_time_index - i;
      auto point = index < 0 ? prev_traj.past_points(index + kMaxPastPointNum)
                             : prev_traj.trajectory_point(index);
      point.set_relative_time(-i * kTrajectoryTimeStep);
      past_points.push_back(point);
    }
  }
  QCHECK_EQ(past_points.size(), kMaxPastPointNum);
  return past_points;
}

ApolloTrajectoryPointProto ComputePlanStartPointAfterReset(
    const std::optional<ApolloTrajectoryPointProto> &prev_reset_planned_point,
    const PoseProto &pose, const Chassis &chassis,
    const MotionConstraintParamsProto &motion_constraint_params,
    const VehicleGeometryParamsProto &vehicle_geom_params,
    const VehicleDriveParamsProto &vehicle_drive_params) {
  ApolloTrajectoryPointProto plan_start_point;
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_s(0.0);
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  const double pose_v = pose.vel_body().x();
  plan_start_point.set_v(std::max(0.0, pose_v));

  if (prev_reset_planned_point.has_value()) {
    constexpr double kFullStopSpeedThreshold = 0.05;  // m/s.
    const bool full_stop =
        prev_reset_planned_point->v() == 0.0 &&
        std::abs(pose.vel_body().x()) < kFullStopSpeedThreshold;
    if (full_stop) {
      plan_start_point.mutable_path_point()->set_kappa(
          prev_reset_planned_point->path_point().kappa());
      plan_start_point.mutable_path_point()->set_lambda(
          prev_reset_planned_point->path_point().lambda());
      plan_start_point.set_a(0.0);
      plan_start_point.set_j(0.0);
      return plan_start_point;
    }
  }

  constexpr double kLowSpeedThreshold = 1.0;  // m/s.
  if (pose_v < kLowSpeedThreshold) {
    // At speed lower than this, we don't trust the measured acceleration and
    // angular velocity. Reset like this are mostly when we're stopped anyway.
    const double steer_angle =
        chassis.has_steering_percentage()
            ? chassis.steering_percentage() / 100.0 *
                  vehicle_drive_params.max_steer_angle() /
                  vehicle_drive_params.steer_ratio()
            : 0.0;  // rad.
    const double kappa =
        std::tan(steer_angle) / vehicle_geom_params.wheel_base();
    plan_start_point.mutable_path_point()->set_kappa(kappa);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(0.0);
    plan_start_point.set_j(0.0);
  } else {
    plan_start_point.mutable_path_point()->set_kappa(pose.ar_smooth().z() /
                                                     pose_v);
    plan_start_point.mutable_path_point()->set_lambda(0.0);
    plan_start_point.set_a(std::clamp(
        pose.accel_body().x(), motion_constraint_params.max_deceleration(),
        motion_constraint_params.max_acceleration()));
    plan_start_point.set_j(0.0);
  }
  return plan_start_point;
}

}  // namespace planner
}  // namespace qcraft
