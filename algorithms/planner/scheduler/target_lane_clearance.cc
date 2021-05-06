#include "onboard/planner/scheduler/target_lane_clearance.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kFrontBackLanePathLength = 100.0;  // m
constexpr double kReferencePathMinSpeed = 1.0;      // m/s.
constexpr double kBoxLateralExpandRatio = 0.6;
constexpr double kBezierCtrlPtRatio = 0.35;

constexpr double kEgoResponseTime = 0.3;  // s

// Aggressiveness = 0.0
const PiecewiseLinearFunction kVehicleSpeedMinBrakePlf_0(
    std::vector<double>{0.0, 10.0, 20.0, 30.0},
    std::vector<double>{1.6, 1.5, 1.3, 0.5});

// Aggressiveness = 1.0
const PiecewiseLinearFunction kVehicleSpeedMinBrakePlf_1(
    std::vector<double>{0.0, 10.0, 20.0, 30.0},
    std::vector<double>{2.5, 2.1, 1.6, 1.0});

// Aggressiveness = 2.0
const PiecewiseLinearFunction kVehicleSpeedMinBrakePlf_2(
    std::vector<double>{0.0, 10.0, 20.0, 30.0},
    std::vector<double>{3.0, 2.6, 1.9, 1.1});

using LanePath = mapping::LanePath;
using SemanticMapManager = mapping::SemanticMapManager;
using VehicleState = RssLongitudialFormulas::VehicleState;

bool ObjectIsOnTargetLane(const AABox2d &slbox) {
  constexpr double kLateralThreshold = 1.0;  // m.
  return std::abs(slbox.center().y()) < kLateralThreshold + slbox.width() * 0.5;
}

// returns a vector of {ra center, tangent}
std::vector<std::pair<Vec2d, Vec2d>> GenerateLcRefPath(
    const FrenetFrame &target_frenet_frame,
    const ApolloTrajectoryPointProto &plan_start_point,
    const PlannerParamsProto &planner_params) {
  const auto &bezier_params =
      planner_params.reference_path_params().bezier_params();
  const double ref_v = std::max(plan_start_point.v(), kReferencePathMinSpeed);
  const double look_ahead_dist =
      std::min(std::max(ref_v * bezier_params.look_ahead_time(),
                        bezier_params.min_look_ahead_dist()),
               target_frenet_frame.end_s());
  const double ctrl_pt_dist = kBezierCtrlPtRatio * look_ahead_dist;

  // Control points for 3rd order Bezier curve.
  const Vec2d p0 = Vec2dFromApolloTrajectoryPointProto(plan_start_point);
  const double ego_s_target = target_frenet_frame.XYToSL(p0).s;
  const auto tan_p0_target =
      target_frenet_frame.InterpolateTangentByS(ego_s_target);
  const double lat0 = tan_p0_target.CrossProd(
      p0 - target_frenet_frame.SLToXY({ego_s_target, 0.0}));
  double lat1 =
      lat0 + ctrl_pt_dist * std::tan(plan_start_point.path_point().theta() -
                                     tan_p0_target.FastAngle());
  if (lat0 * lat1 < 0.0) lat1 = 0.0;  // Never cross the target lane.
  const Vec2d p1 =
      target_frenet_frame.SLToXY({ego_s_target + ctrl_pt_dist, lat1});
  const Vec2d p2 = target_frenet_frame.SLToXY(
      {ego_s_target + look_ahead_dist - ctrl_pt_dist, 0.0});
  const Vec2d p3 =
      target_frenet_frame.SLToXY({ego_s_target + look_ahead_dist, 0.0});
  const Vec2d p_0_1 = p1 - p0;
  const Vec2d p_1_2 = p2 - p1;
  const Vec2d p_2_3 = p3 - p2;

  // Boxes during lane change.
  std::vector<std::pair<Vec2d, Vec2d>> res;
  res.reserve(kTrajectorySteps);
  const double dt = (ref_v * kTrajectoryTimeStep) / look_ahead_dist;
  const int lc_point_num = std::min(
      FloorToInt(bezier_params.look_ahead_time() / kTrajectoryTimeStep),
      kTrajectorySteps);
  for (int i = 0; i < lc_point_num; ++i) {
    const double interp_t = i * dt;
    Vec2d center =
        Cube(1 - interp_t) * p0 + 3 * Sqr(1 - interp_t) * interp_t * p1 +
        3 * (1 - interp_t) * Sqr(interp_t) * p2 + Cube(interp_t) * p3;
    Vec2d tangent =
        (3 * Sqr(1 - interp_t) * p_0_1 + 6 * (1 - interp_t) * interp_t * p_1_2 +
         3 * Sqr(interp_t) * p_2_3)
            .normalized();
    res.push_back({std::move(center), std::move(tangent)});
  }

  // If less than 10s, fill in the rest boxes.
  const double s_per_step = ref_v * kTrajectoryTimeStep;
  for (int i = lc_point_num; i < kTrajectorySteps; ++i) {
    const double sample_s =
        ego_s_target + look_ahead_dist + (i - lc_point_num) * s_per_step;
    if (sample_s > target_frenet_frame.end_s()) break;

    res.push_back({target_frenet_frame.SLToXY({sample_s, 0.0}),
                   target_frenet_frame.InterpolateTangentByS(sample_s)});
  }

  return res;
}

std::vector<Box2d> GenerateBoxesFromPath(
    const std::vector<std::pair<Vec2d, Vec2d>> &path,
    const VehicleGeometryParamsProto &vehicle_geom) {
  std::vector<Box2d> boxes;
  boxes.reserve(path.size());
  const double ra_to_center =
      vehicle_geom.front_edge_to_center() - 0.5 * vehicle_geom.length();

  const double half_length = vehicle_geom.length() * 0.5;
  const double half_width = vehicle_geom.width() * 0.5;
  for (const auto &point : path) {
    boxes.emplace_back(half_length, half_width,
                       point.first + ra_to_center * point.second, point.second);
  }

  return boxes;
}

bool PathHasOverlap(absl::Span<const Box2d> path_ego,
                    const SpacetimeObjectTrajectory &path_other,
                    double lat_ext_ratio) {
  // Handle stationary trajectory.
  if (path_other.is_stationary()) {
    auto box_other = path_other.bounding_box();
    box_other.LateralExtendByRatio(lat_ext_ratio);
    for (const auto &box_ego : path_ego) {
      if (box_ego.HasOverlap(box_other)) {
        return true;
      }
    }
    return false;
  }

  // Handle moving trajectory.
  for (const auto &box_ego : path_ego)
    for (const auto &state_other : path_other.states()) {
      auto box_other = state_other.box;
      box_other.LateralExtendByRatio(lat_ext_ratio);
      if (box_ego.HasOverlap(box_other)) {
        return true;
      }
    }

  return false;
}

bool PathIntersectsWithLanePath(const SpacetimeObjectTrajectory &path,
                                const FrenetFrame &frenet_frame) {
  // Handle stationary trajectory.
  if (path.is_stationary()) {
    if (std::abs(frenet_frame.XYToSL(path.pose().pos()).l) <=
        kDefaultHalfLaneWidth) {
      return true;
    }
    return false;
  }

  // Handle moving trajectory.
  const auto states = path.states();
  for (int i = 0; i < states.size(); i += 5) {
    if (std::abs(frenet_frame.XYToSL(states[i].traj_point->pos()).l) <=
        kDefaultHalfLaneWidth) {
      return true;
    }
  }
  return false;
}

AABox2d ProjectBoxOnSL(const Box2d &box_xy, const FrenetFrame &frenet_frame) {
  const auto center_sl = frenet_frame.XYToSL(box_xy.center());
  const auto tangent = Vec2d::FastUnitFromAngle(
      box_xy.heading() -
      frenet_frame.InterpolateTangentByS(center_sl.s).FastAngle());

  return Box2d(Vec2d(center_sl.s, center_sl.l), std::move(tangent),
               box_xy.length(), box_xy.width())
      .GetAABox();
}

double ProjectVelocityOnS(Vec2d pos, double v, double theta,
                          const FrenetFrame &frenet_frame) {
  const Vec2d frenet_tangent = frenet_frame.InterpolateTangentByXY(pos);
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);

  return std::max(0.0, v * frenet_tangent.dot(tangent));
}

}  // namespace

double ComputeObjectResponseTime(double aggr, double accel) {
  constexpr double kObjectFastAccelThreshold = 1.0;  // m/s^2

  if (accel > kObjectFastAccelThreshold) {
    const PiecewiseLinearFunction kFastAccelResponseTimePlf(
        std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{2.0, 1.0, 0.0});
    return kFastAccelResponseTimePlf(aggr);
  }

  constexpr double kObjectAccelThreshold = 0.2;  // m/s^2
  if (accel > kObjectAccelThreshold) {
    const PiecewiseLinearFunction kAccelResponseTimePlf(
        std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{1.0, 0.3, 0.0});
    return kAccelResponseTimePlf(aggr);
  }

  const PiecewiseLinearFunction kObjectYieldResponseTimePlf(
      std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{0.5, 0.1, 0.0});
  return kObjectYieldResponseTimePlf(aggr);
}

double ComputeVehicleMinBrake(double v, double aggr) {
  const PiecewiseLinearFunction min_brake_plf(
      std::vector<double>{0.0, 1.0, 2.0},
      std::vector<double>{kVehicleSpeedMinBrakePlf_0(v),
                          kVehicleSpeedMinBrakePlf_1(v),
                          kVehicleSpeedMinBrakePlf_2(v)});

  return min_brake_plf(aggr);
}

ClearanceCheckOutput::ObjectDecision DecideClearanceForMovingObject(
    const VehicleState &ego_state, const VehicleState &obj_state,
    double aggr_factor) {
  const bool ego_at_front =
      ego_state.vehicle_box.center().x() > obj_state.vehicle_box.center().x();

  const int s_sign = ego_at_front ? 1 : -1;
  const double ego_s = ego_state.vehicle_box.center_x() -
                       s_sign * ego_state.vehicle_box.half_length();
  const double obj_s = obj_state.vehicle_box.center_x() +
                       s_sign * obj_state.vehicle_box.half_length();
  const double lon_dist = std::max(0.0, s_sign * (ego_s - obj_s));

  const double min_safe_dist =
      aggr_factor > 1.0 ? 0.1  // Can be more aggressive when executing lc.
                        : RssLongitudialFormulas::kMinSafeDistance;

  VLOG(4) << ego_state.DebugString();
  VLOG(4) << obj_state.DebugString();

  if (lon_dist < min_safe_dist) {
    // Object in dead zone, return unsafe.
    VLOG(3) << "Object in dead zone";
    return ClearanceCheckOutput::LC_UNSAFE;
  }

  if (aggr_factor <= 1.0 && !ego_at_front) {
    if (obj_state.type == OT_PEDESTRIAN) {
      // Pedestrian in front of ego on the target lane, return unsafe.
      VLOG(3) << "Ped is on target lane";
      return ClearanceCheckOutput::LC_UNSAFE;
    }
  }

  constexpr double kMinTTC = 3.0;  // seconds
  if (ego_at_front &&
      (obj_state.current_v - ego_state.current_v) * kMinTTC > lon_dist) {
    VLOG(3) << "TTC < " << kMinTTC;
    return ClearanceCheckOutput::LC_UNSAFE;
  }

  constexpr double kFastMovingSpeed = 5.0;  // m/s
  if (!ego_at_front && obj_state.current_v > kFastMovingSpeed &&
      (ego_state.current_v - obj_state.current_v) * kMinTTC > lon_dist) {
    VLOG(3) << "Front object blocking.";
    return ClearanceCheckOutput::LC_UNSAFE;
  }

  // TODO(boqian): handle objects moving in the opposite direction
  const double safe_dist = std::max(
      min_safe_dist,
      ego_at_front
          ? RssLongitudialFormulas::SafeLongitudinalDistanceSameDirection(
                ego_state, obj_state)
          : RssLongitudialFormulas::SafeLongitudinalDistanceSameDirection(
                obj_state, ego_state));

  VLOG(3) << (ego_at_front ? "ego front" : "ego back")
          << ", lon dist: " << lon_dist << ", "
          << "safe dist: " << safe_dist;
  if (lon_dist < safe_dist) return ClearanceCheckOutput::LC_UNSAFE;

  return ego_at_front ? ClearanceCheckOutput::LC_LEAD
                      : ClearanceCheckOutput::LC_FOLLOW;
}

// aggr_factor: a parameter in [0, 2] indicating how aggressive the ego vehicle
// will be during the lc process. It should be in [0, 1] when initiating lc,
// in (1, 2) when executing lc, and equal to 2 when already on target lane.
absl::StatusOr<ClearanceCheckOutput> CheckTargetLaneClearance(
    const PlannerSemanticMapManager &psmm, const LanePath &target_lane_path,
    const ApolloTrajectoryPointProto &plan_start_point,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const absl::flat_hash_set<std::string> &stalled_objects,
    const VehicleGeometryParamsProto &vehicle_geom,
    const PlannerParamsProto &planner_params, double aggr_factor,
    ThreadPool *thread_pool) {
  SCOPED_QTRACE("CheckTargetLaneClearance");

  VLOG(1) << "aggressiveness: " << aggr_factor;

  // 0. Limit the length of target lane path.
  const auto target_lane_path_ext = BackwardExtendLanePath(
      *psmm.semantic_map_manager(),
      target_lane_path.BeforeArclength(kFrontBackLanePathLength),
      kFrontBackLanePathLength);
  ASSIGN_OR_RETURN(const auto target_frenet_frame,
                   BuildKdTreeFrenetFrame(mapping::SampleLanePathPoints(
                       *psmm.semantic_map_manager(), target_lane_path_ext)));

  // 1. Generate pseudo trajectory (as boxes) for ego lane change.
  const auto ref_boxes = GenerateBoxesFromPath(
      GenerateLcRefPath(target_frenet_frame, plan_start_point, planner_params),
      vehicle_geom);
  const double box_lat_ext_ratio =
      1.0 + kBoxLateralExpandRatio * (2.0 - aggr_factor);

  // 2. Check stationary objects for possible collision.
  const auto stationary_trajs = st_traj_mgr.stationary_object_trajs();
  // NOTE(lidong): Don't use std::vector<bool> here. It might cause data racing
  // problem as std::vector<bool> has shared bytes.
  std::vector<int> stationary_object_overlap_bit(stationary_trajs.size(), -1);
  ParallelFor(0, stationary_trajs.size(), thread_pool, [&](int i) {
    const auto &planner_object = *stationary_trajs[i]->planner_object();
    if (planner_object.type() == ObjectType::OT_VEGETATION) {
      return;  // Ignore vegetation.
    }
    if (!stalled_objects.contains(planner_object.id())) {
      // Only consider objects here instead of all stationary objects.
      return;
    }
    // If not on the target lane.
    const auto obj_sl = target_frenet_frame.XYToSL(planner_object.pose().pos());
    if (obj_sl.s < target_frenet_frame.start_s() ||
        target_frenet_frame.end_s() < obj_sl.s ||
        std::abs(obj_sl.l) > kDefaultHalfLaneWidth) {
      return;
    }
    stationary_object_overlap_bit[i] = 0;  // Mark as relevant and stalled.
    auto obj_box = planner_object.bounding_box();
    obj_box.LateralExtendByRatio(box_lat_ext_ratio);
    for (const auto &box : ref_boxes) {
      if (obj_box.HasOverlap(box)) {
        stationary_object_overlap_bit[i] = 1;  // Mark the collision.
        break;
      }
    }
  });
  for (int i = 0; i < stationary_trajs.size(); ++i) {
    if (stationary_object_overlap_bit[i] == 1) {
      return absl::CancelledError(
          absl::StrFormat("Target lane not clear - stalled object %s.",
                          stationary_trajs[i]->planner_object()->id()));
    }
  }

  // If already on target lane, skip checking moving objects.
  if (aggr_factor > 2.0) {
    ClearanceCheckOutput res;
    res.pseudo_traj.reserve(ref_boxes.size());
    for (const auto &box : ref_boxes) {
      res.pseudo_traj.push_back(box.center());
    }
    return res;
  }

  // 3. Check all objects for possible collision.
  double speed_limit =
      planner_params.motion_constraint_params().default_speed_limit();
  for (const auto id : target_lane_path_ext.lane_ids()) {
    speed_limit = std::min(speed_limit, psmm.QueryLaneSpeedLimitById(id));
  }

  VehicleState ego_state;
  ego_state.current_v = ProjectVelocityOnS(
      Vec2dFromApolloTrajectoryPointProto(plan_start_point),
      plan_start_point.v(), plan_start_point.path_point().theta(),
      target_frenet_frame);
  ego_state.max_v = speed_limit;
  ego_state.response_time = kEgoResponseTime;
  ego_state.min_brake = kEgoMinBrakePlf(aggr_factor);
  ego_state.max_brake = kEgoMaxBrakePlf(aggr_factor);
  ego_state.type = OT_VEHICLE;
  ego_state.max_accel = 0.0;
  ego_state.vehicle_box =
      ProjectBoxOnSL(ref_boxes.front(), target_frenet_frame);
  ego_state.id = "ego";

  const auto st_trajs = st_traj_mgr.trajectories();

  std::vector<ClearanceCheckOutput::ObjectDecision> obj_traj_lc_decision(
      st_trajs.size(), ClearanceCheckOutput::LC_IRRELEVANT);

  ParallelFor(0, st_trajs.size(), thread_pool, [&](int i) {
    const auto *traj = st_trajs[i];
    const auto &planner_object = *traj->planner_object();
    if (planner_object.type() == ObjectType::OT_VEGETATION) {
      return;  // Ignore vegetation.
    }
    if (stalled_objects.contains(planner_object.id())) {
      // Only consider non-object stationary objects here.
      return;
    }
    if (!(PathHasOverlap(ref_boxes, *traj, box_lat_ext_ratio) &&
          PathIntersectsWithLanePath(*traj, target_frenet_frame))) {
      return;  // No overlap with ref path or target lane.
    }

    const auto traj_states = traj->states();
    const auto &state = traj_states.front();
    auto obj_box = ProjectBoxOnSL(state.box, target_frenet_frame);

    // BANDAID(weijun): Ignore vehicle which is currently not on target lane.
    if (!ObjectIsOnTargetLane(obj_box)) {
      return;
    }

    VehicleState obj_state;
    obj_state.current_v =
        ProjectVelocityOnS(state.traj_point->pos(), state.traj_point->v(),
                           state.traj_point->theta(), target_frenet_frame);
    obj_state.max_v = std::numeric_limits<double>::max();
    obj_state.min_brake =
        ComputeVehicleMinBrake(obj_state.current_v, aggr_factor);
    obj_state.max_brake = kVehicleMaxBrakePlf(aggr_factor);
    obj_state.type = traj->planner_object()->type();
    obj_state.vehicle_box = std::move(obj_box);
    obj_state.max_accel =
        aggr_factor >= 1.0 ? 0.0 : std::max(0.0, state.traj_point->a());
    obj_state.response_time =
        ComputeObjectResponseTime(aggr_factor, obj_state.max_accel);
    obj_state.id = planner_object.id();

    obj_traj_lc_decision[i] =
        DecideClearanceForMovingObject(ego_state, obj_state, aggr_factor);
  });

  for (int i = 0; i < st_trajs.size(); ++i) {
    if (obj_traj_lc_decision[i] == ClearanceCheckOutput::LC_UNSAFE) {
      return absl::CancelledError(absl::StrFormat(
          "Target lane not clear - trajectory %s.", st_trajs[i]->traj_id()));
    }
  }

  // OK
  ClearanceCheckOutput res;
  res.pseudo_traj.reserve(ref_boxes.size());
  for (const auto &box : ref_boxes) {
    res.pseudo_traj.push_back(box.center());
  }
  for (int i = 0; i < st_trajs.size(); ++i) {
    res.objects_decision[st_trajs[i]->traj_id()] = obj_traj_lc_decision[i];
  }
  for (int i = 0; i < stationary_trajs.size(); ++i) {
    if (stationary_object_overlap_bit[i] == 0) {
      res.objects_decision[stationary_trajs[i]->traj_id()] =
          ClearanceCheckOutput::LC_OVERTAKE;
    }
  }
  return res;
}

}  // namespace qcraft::planner
