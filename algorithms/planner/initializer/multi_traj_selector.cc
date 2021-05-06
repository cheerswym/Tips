#include "onboard/planner/initializer/multi_traj_selector.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "absl/status/statusor.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

// Lane change.
constexpr double kMoveAlongPathPercentageThreshold =
    0.5;                                      // ! Not sure about this.
constexpr double kWillEnterSRangeTime = 1.0;  // s.
constexpr int kMultiTrajMaxSize = 4;

// RSS constants. (Copied from target_lane_clearance.h)
using VehicleState = RssLongitudialFormulas::VehicleState;
const PiecewiseLinearFunction kEgoMinBrakePlf(
    std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{1.2, 1.7, 2.5});
const PiecewiseLinearFunction kEgoMaxBrakePlf(
    std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{0.95, 1.2, 1.4});
const PiecewiseLinearFunction kVehicleMaxBrakePlf(
    std::vector<double>{0.0, 1.0, 2.0}, std::vector<double>{0.95, 0.8, 0.6});

// Threshold to determine whether obj is along path.
constexpr double kLateralThreshold = 1.0;
constexpr double kEgoResponseTime = 0.3;  // s.
constexpr double kMinTTC = 3.0;           // s.

// Costs.
// constexpr double kTTCCheckCost = 1000.0;
// constexpr double kDeadZoneCost = 1000.0;

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

// Get aggressiveness factor from the ego lateral distance to drive passage
// reference line. [0, 1] when initiating LC, (1, 2) when executing, equal to 2
// when on target lane. Cannot directly use lane change stage because we are
// evaluating a future ego state. For this selector, we are evaluating the time
// when vehicle actually enters the target lane, so aggr_factor = 2 for this
// selector.
const PiecewiseLinearFunction kLateralDistToAggrFactorPlf(
    std::vector<double>{kDefaultLaneWidth, kLateralThreshold},
    std::vector<double>{1.0, 2.0});

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

bool IsLeadingObjectType(ObjectType type) {
  switch (type) {
    case OT_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
      return true;
    case OT_UNKNOWN_STATIC:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return false;
  }
}

double ComputeVehicleMinBrake(double v, double aggr) {
  const PiecewiseLinearFunction min_brake_plf(
      std::vector<double>{0.0, 1.0, 2.0},
      std::vector<double>{kVehicleSpeedMinBrakePlf_0(v),
                          kVehicleSpeedMinBrakePlf_1(v),
                          kVehicleSpeedMinBrakePlf_2(v)});

  return min_brake_plf(aggr);
}

double ComputeLongitudinalDistance(const AABox2d& ego, const AABox2d& object,
                                   bool ego_at_front) {
  const int s_sign = ego_at_front ? 1 : -1;
  const double ego_with_box_s = ego.center_x() - s_sign * ego.half_length();
  const double obj_with_box_s =
      object.center_x() + s_sign * object.half_length();
  return s_sign * (ego_with_box_s - obj_with_box_s);
}

absl::StatusOr<int> GetEnteringTargetLaneTrajPoint(
    const FrenetFrame* frenet_frame,
    const std::vector<ApolloTrajectoryPointProto>& trajectory, double sdc_width,
    FrenetCoordinate* enter_target_lane_sl) {
  for (int i = 0; i < trajectory.size(); i++) {
    const auto& point = trajectory[i];
    const auto sl = frenet_frame->XYToSL(
        Vec2d(point.path_point().x(), point.path_point().y()));
    if (std::fabs(sl.l) < kDefaultHalfLaneWidth + sdc_width * 0.5) {
      *enter_target_lane_sl = sl;
      return i;
    }
  }
  return absl::NotFoundError(
      "Current evaluated trajectory does not successfully enter target lane.");
}

double ProjectVelocityOnTargetLane(const FrenetFrame& frenet_frame, Vec2d pos,
                                   double v, double theta) {
  const Vec2d frenet_tangent = frenet_frame.InterpolateTangentByXY(pos);
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
  return std::max(0.0, v * frenet_tangent.dot(tangent));
}

AABox2d ProjectBoxOnTargetLane(const FrenetFrame& frenet_frame,
                               const Box2d& box,
                               const FrenetCoordinate* box_center_sl) {
  FrenetCoordinate center_sl;
  if (box_center_sl == nullptr) {
    // If sl is not yet calculated before, calculate here.
    // Normally, ego's sl should already be calculated.
    center_sl = frenet_frame.XYToSL(box.center());
  } else {
    center_sl = *box_center_sl;
  }
  const auto tangent = Vec2d::FastUnitFromAngle(
      box.heading() -
      frenet_frame.InterpolateTangentByS(center_sl.s).FastAngle());
  return Box2d(Vec2d(center_sl.s, center_sl.l), std::move(tangent),
               box.length(), box.width())
      .GetAABox();
}

bool IsEgoAtFront(const AABox2d& ego_box, const AABox2d& vehicle_box) {
  return ego_box.center().x() > vehicle_box.center().x();
}

double GetEgoSpeedLimit(const DrivePassage& passage,
                        const PlannerParamsProto& planner_params,
                        const mapping::LanePath& lane_path,
                        const Vec2d& ego_pos) {
  const auto speed_limit_or = passage.QuerySpeedLimitAt(ego_pos);
  double speed_limit =
      planner_params.motion_constraint_params().default_speed_limit();
  if (speed_limit_or.ok()) {
    speed_limit = speed_limit_or.value();
  } else {
    for (const auto& id : lane_path.lane_ids()) {
      double lane_speed_limit =
          passage.planner_semantic_map_manager()->QueryLaneSpeedLimitById(id);
      speed_limit = std::min(speed_limit, lane_speed_limit);
    }
  }
  return speed_limit;
}

VehicleState BuildEgoState(double aggr_factor,
                           const FrenetFrame& target_frenet_frame,
                           const Vec2d& ego_pos,
                           const ApolloTrajectoryPointProto& ego_point,
                           const DrivePassage& passage,
                           const mapping::LanePath& lane_path,
                           const VehicleGeometryParamsProto& vehicle_geom,
                           const PlannerParamsProto& planner_params,
                           const FrenetCoordinate* enter_target_state_sl) {
  VehicleState ego_state;
  ego_state.current_v =
      ProjectVelocityOnTargetLane(target_frenet_frame, ego_pos, ego_point.v(),
                                  ego_point.path_point().theta());
  VLOG(5) << "ego current v projected on target lane is " << ego_state.current_v
          << " heading: " << ego_point.path_point().theta();
  ego_state.max_v =
      GetEgoSpeedLimit(passage, planner_params, lane_path, ego_pos);
  ego_state.response_time = kEgoResponseTime;
  ego_state.min_brake = kEgoMinBrakePlf(aggr_factor);
  ego_state.max_brake = kEgoMaxBrakePlf(aggr_factor);
  ego_state.type = OT_VEHICLE;
  // Use ego's current acceleration.
  ego_state.max_accel = std::max<double>(0.0, ego_point.a());
  Box2d ego_box =
      GetAvBox(ego_pos, ego_point.path_point().theta(), vehicle_geom);
  ego_state.vehicle_box = ProjectBoxOnTargetLane(target_frenet_frame, ego_box,
                                                 enter_target_state_sl);
  ego_state.id = "ego";
  return ego_state;
}

VehicleState BuildObjectState(const FrenetFrame& target_frenet_frame,
                              const SpacetimeObjectState& vehicle_state,
                              const PlannerObject& planner_object,
                              double aggr_factor, AABox2d obj_aa_box) {
  VehicleState obj_state;
  obj_state.current_v = ProjectVelocityOnTargetLane(
      target_frenet_frame, vehicle_state.traj_point->pos(),
      vehicle_state.traj_point->v(), vehicle_state.traj_point->theta());
  obj_state.max_v = std::numeric_limits<double>::max();
  obj_state.min_brake =
      ComputeVehicleMinBrake(obj_state.current_v, aggr_factor);
  obj_state.max_brake = kVehicleMaxBrakePlf(aggr_factor);
  obj_state.type = planner_object.type();
  obj_state.vehicle_box = std::move(obj_aa_box);
  obj_state.max_accel = std::max(0.0, vehicle_state.traj_point->a());
  obj_state.response_time =
      ComputeObjectResponseTime(aggr_factor, obj_state.max_accel);
  obj_state.id = planner_object.id();
  return obj_state;
}

enum {
  DP_LEADING_OBJ_COST = 12,
  DP_FINAL_LON_PROGRESS_COST = 13,
};

absl::StatusOr<double> GetDpLeadingObjectCost(const SingleTrajInfo& traj_info) {
  const auto& search_costs = traj_info.search_costs;
  if (search_costs.size() == 0) {
    return absl::NotFoundError(
        "search_costs size is zero. Output is a stationary motion.");
  }
  if (traj_info.last_edge_index.value() >= search_costs.size()) {
    return absl::NotFoundError(
        "Quried trajectory costs not stored in search_costs. Output is a "
        "stationary motion.");
  }
  const auto& feature_costs =
      search_costs[traj_info.last_edge_index].feature_cost;
  return feature_costs[DP_LEADING_OBJ_COST];
}

absl::StatusOr<double> GetFinalLongitudinalProgressCost(
    const SingleTrajInfo& traj_info) {
  const auto& search_costs = traj_info.search_costs;
  if (search_costs.size() == 0) {
    return absl::NotFoundError(
        "search_costs size is zero. Output is a stationary motion.");
  }
  if (traj_info.last_edge_index.value() >= search_costs.size()) {
    return absl::NotFoundError(
        "Quried trajectory costs not stored in search_costs. Output is a "
        "stationary motion.");
  }
  const auto& feature_costs =
      search_costs[traj_info.last_edge_index].feature_cost;
  VLOG(5) << absl::StrJoin(feature_costs, ",");
  return feature_costs[DP_FINAL_LON_PROGRESS_COST];
}

void CheckTTCAndDeadZone(
    const VehicleState& ego_state, const VehicleState& obj_state,
    double lon_dist, double min_safe_dist, bool ego_at_front,
    InitializerSelectorDebugProto::TrajectoryEvalInfo* eval_info) {
  if ((obj_state.current_v - ego_state.current_v) * kMinTTC >
      std::max<double>(lon_dist, 0.0)) {
    eval_info->set_ttc_check(false);
  } else {
    eval_info->set_ttc_check(true);
  }

  // 3.3. Compare the longitudinal distance with required safe dist
  // and min safe dist.
  const double required_safe_dist_rss =
      ego_at_front
          ? RssLongitudialFormulas::SafeLongitudinalDistanceSameDirection(
                ego_state, obj_state)
          : RssLongitudialFormulas::SafeLongitudinalDistanceSameDirection(
                obj_state, ego_state);
  const double required_safe_dist =
      std::max(min_safe_dist, required_safe_dist_rss);
  VLOG(5) << "ego_at_front ? " << ego_at_front
          << " longitudinal distance: " << lon_dist
          << " required safe dist: " << required_safe_dist
          << " required safe dist by rss: " << required_safe_dist_rss;
  // We should choose the max safe distance.
  if (std::max<double>(0.0, lon_dist) < required_safe_dist) {
    // lon_dist smaller than min_safe_dist, set safe distance to lowest so
    // selector ranks it low.
    eval_info->set_dead_zone_check(false);
  } else {
    eval_info->set_dead_zone_check(true);
  }
}

struct LeadingObjectTrajectoryInfo {
  // Project potential leading object trajectory onto drive passage.
  double s;
  int s_available_idx;
  absl::string_view traj_id;
};

bool LaterallyEnterLaneBoundary(const FrenetBox& fbox,
                                const PathSlBoundary& path_sl, bool lc_left) {
  // Get sl boundary at s_min and s_max.
  // Returned pair: first: right_l, second: left_l.
  const auto l_s_min = path_sl.QueryBoundaryL(fbox.s_min);
  const auto l_s_max = path_sl.QueryBoundaryL(fbox.s_max);
  const auto l_center =
      path_sl.QueryReferenceCenterL(0.5 * (fbox.s_min + fbox.s_max));
  const double l_left_avg = 0.5 * (l_s_min.second + l_s_max.second);
  const double l_right_avg = 0.5 * (l_s_min.first + l_s_max.first);
  // As long as fbox enters lane boundary (outer), we consider it valid.
  return lc_left ? fbox.l_min < l_left_avg && fbox.l_max > l_center
                 : fbox.l_max > l_right_avg && fbox.l_min < l_center;
}

bool CurrentlyNearTargetLanePath(const SpacetimeObjectTrajectory& traj,
                                 const PathSlBoundary& path_sl,
                                 const DrivePassage& drive_passage,
                                 bool lc_left,
                                 LeadingObjectTrajectoryInfo* traj_info) {
  FUNC_QTRACE();
  // Loosely find objects that are moving near target lane path, only check
  // current position.
  const auto& states = traj.states();
  ASSIGN_OR_RETURN(const auto fbox,
                   drive_passage.QueryFrenetBoxAt(states.front().box), false);

  if (LaterallyEnterLaneBoundary(fbox, path_sl, lc_left)) {
    traj_info->s_available_idx = 0;
    traj_info->traj_id = traj.traj_id();
    traj_info->s = fbox.s_min;
    return true;
  }
  return false;
}

bool MovingAlongPath(const SpacetimeObjectTrajectory& traj,
                     const PathSlBoundary& path_sl,
                     const DrivePassage& drive_passage, bool lc_left,
                     LeadingObjectTrajectoryInfo* traj_info) {
  FUNC_QTRACE();
  // Use prediction to find possible lane change targets.
  const auto& states = traj.states();
  constexpr int kEvalStep = 1;
  int s_available_idx = -1;
  int along_path_states = 0;
  int evaluate_states = 0;
  double s = 0.0;
  for (int i = 0; i < states.size(); i += kEvalStep) {
    const auto& object_box = states[i].box;
    ASSIGN_OR_CONTINUE(const auto fbox,
                       drive_passage.QueryFrenetBoxAt(object_box));
    evaluate_states++;
    if (s_available_idx == -1) {
      // Record the first projectable state onto drive passage.
      s_available_idx = i;
      s = 0.5 * (fbox.s_min + fbox.s_max);
    }
    if (LaterallyEnterLaneBoundary(fbox, path_sl, lc_left)) {
      along_path_states++;
    }
  }
  const double on_path_ratio = static_cast<double>(along_path_states) /
                               static_cast<double>(evaluate_states);
  bool decision = false;
  if (on_path_ratio > kMoveAlongPathPercentageThreshold &&
      s_available_idx <= static_cast<int>(kWillEnterSRangeTime *
                                          (1.0 / kTrajectoryTimeStep))) {
    decision = true;
    traj_info->s_available_idx = s_available_idx;
    traj_info->traj_id = traj.traj_id();
    traj_info->s = s;
  }
  return decision;
}

void AddBlockingStaticObjToLeadingObjs(
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ConstraintProto::LeadingObjectProto* blocking_static_obj,
    InitializerSearchConfig::LeadingObjectConfig* leading_object_config) {
  if (blocking_static_obj != nullptr &&
      !stalled_objects.contains(
          SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
              blocking_static_obj->traj_id()))) {
    VLOG(3) << "Adding non-stalled blocking static: "
            << blocking_static_obj->traj_id();
    leading_object_config->add_front(blocking_static_obj->traj_id());
  }
}
void GetLeadingObjectConfig(
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<std::string>* front,
    const absl::flat_hash_set<std::string>* rear,
    InitializerSearchConfig::LeadingObjectConfig* config) {
  if (front != nullptr && front->size() != 0) {
    for (const auto& traj_id : *front) {
      if (stalled_objects.contains(
              SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
                  traj_id))) {
        // Considered potential leading object is actually a stalled one, add to
        // rear.
        config->add_rear(traj_id);
        continue;
      }
      config->add_front(traj_id);
    }
  }
  if (rear != nullptr && rear->size() != 0) {
    for (const auto& object : *rear) {
      config->add_rear(object);
    }
  }
}

void EvaluateSingleTrajectory(
    int traj_idx, const SingleTrajInfo& traj_info,
    const std::vector<std::string>& all_leading_objs,
    const FrenetFrame& target_frenet_frame,
    const PlannerParamsProto& planner_params,
    const SpacetimeTrajectoryManager* st_traj_mgr, const DrivePassage* passage,
    const mapping::LanePath& lane_path,
    const VehicleGeometryParamsProto* vehicle_geom,
    InitializerSelectorDebugProto::TrajectoryEvalInfo* eval_info) {
  const auto& trajectory = traj_info.traj_points;

  // Get trajectory point where ego enters target lane.
  FrenetCoordinate enter_target_state_sl;
  const auto enter_target_lane_idx_or = GetEnteringTargetLaneTrajPoint(
      &target_frenet_frame, trajectory, vehicle_geom->width(),
      &enter_target_state_sl);
  // If we cannot find enter target lane idx, set it to the last point of the
  // trajectory.
  if (!enter_target_lane_idx_or.ok()) {
    auto callback = [&traj_info, &traj_idx](QEvent* qevent) {
      for (const auto& lead_obj : traj_info.lead_objs) {
        qevent->AddField("trajectory_idx", traj_idx)
            .AddField("leading_obj_traj_id", lead_obj);
      }
    };
    QEVENT_EVERY_N_SECONDS("changqing", "initializer_multi_traj_not_enter_lane",
                           20, callback);
    // If the evaluated trajectory does not enter target lane, do not evaluate
    // its longitudinal relations with the objects on the target lane.
    return;
  }

  const auto& ego_point = trajectory[*enter_target_lane_idx_or];
  const Vec2d ego_pos(ego_point.path_point().x(), ego_point.path_point().y());
  VLOG(5) << "ego enters lane at " << ego_point.relative_time()
          << " s with velocity " << ego_point.v() << " m/s.";

  // Build ego vehicle state.
  const double aggr_factor = 2.0;
  const VehicleState ego_state = BuildEgoState(
      aggr_factor, target_frenet_frame, ego_pos, ego_point, *passage, lane_path,
      *vehicle_geom, planner_params, &enter_target_state_sl);

  // Only consider the vehicle behind ego when it enters the target lane.
  // Iterate through objects and update the final cost for this
  // planned trajectory.
  const double min_safe_dist =
      aggr_factor > 1.0 ? 0.1 : RssLongitudialFormulas::kMinSafeDistance;
  double min_long_dist = std::numeric_limits<double>::max();
  std::string obj_traj_id_causing_min_lon_dist;
  for (const auto& obj_traj_id : all_leading_objs) {
    // 1. Get object AABox along target lane.
    const auto* obj_traj = st_traj_mgr->FindTrajectoryById(obj_traj_id);
    const auto obj_states = obj_traj->states();
    const int vehicle_state_idx = std::clamp(
        *enter_target_lane_idx_or, 0, static_cast<int>(obj_states.size()) - 1);
    VLOG(5) << "For " << obj_traj_id << ", it is at " << vehicle_state_idx
            << " th idx when ego enters the lane.";
    const auto& state = obj_states[vehicle_state_idx];
    const auto& obj_state_box = state.box;
    const auto* planner_object = obj_traj->planner_object();
    const auto obj_aa_box =
        ProjectBoxOnTargetLane(target_frenet_frame, obj_state_box, nullptr);

    // 2. Determine longitudinal relationship between ego and vehicle.
    const bool ego_at_front = IsEgoAtFront(ego_state.vehicle_box, obj_aa_box);
    const double lon_dist = ComputeLongitudinalDistance(
        ego_state.vehicle_box, obj_aa_box, ego_at_front);

    if (!ego_at_front) {
      continue;
    }

    // 3. Continue to evaluate whether ego and vehicle has enough safe
    // distance when the vehicle is behind ego.

    // 3.1 Build vehicle state.
    const VehicleState obj_state =
        BuildObjectState(target_frenet_frame, state, *planner_object,
                         aggr_factor, std::move(obj_aa_box));

    // 3.2 Check TTC and DeadZone condition.
    CheckTTCAndDeadZone(ego_state, obj_state, lon_dist, min_safe_dist,
                        ego_at_front, eval_info);

    // Update trajectory min_cost based on the minimum longitudinal
    // distance.
    if (lon_dist < min_long_dist) {
      min_long_dist = lon_dist;
      obj_traj_id_causing_min_lon_dist = obj_traj_id;
    }
  }
  // After evaluating all object state when ego enters the target lane. We
  // have a min longitudinal distance for each traj and the traj_id that
  // causes it.
  eval_info->set_min_lon_dist(min_long_dist);
  eval_info->set_object_causing_min_lon(obj_traj_id_causing_min_lon_dist);
}

}  // namespace

std::vector<InitializerSearchConfig> BuildSearchConfig(
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ConstraintManager& constraint_manager,
    const std::optional<ClearanceCheckOutput>* lc_clearance,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const ConstraintProto::LeadingObjectProto* blocking_static_obj,
    const GeometryGraph& geom_graph,
    const VehicleGeometryParamsProto& vehicle_geom, bool is_lane_change,
    bool lc_left, bool lc_multiple_traj) {
  SCOPED_QTRACE("BuildSearchConfig");
  const double geom_max_accumulated_s = geom_graph.GetMaxAccumulatedS();
  std::vector<absl::flat_hash_set<std::string>> groups_leading_objs;
  if (lc_multiple_traj && is_lane_change) {
    // If current lateral offset greater than a value, consider we need lane
    // change or lane borrow.
    VLOG(3) << "lc_multiple_traj true: constructing leading object groups";
    const auto& considered_trajectories = st_traj_mgr.trajectories();
    std::vector<LeadingObjectTrajectoryInfo> filtered_trajs;
    for (const auto& traj_ptr : considered_trajectories) {
      const auto& traj = *traj_ptr;
      VLOG(3) << "Consider traj: " << traj.traj_id();
      const auto object_type = traj.planner_object()->type();
      LeadingObjectTrajectoryInfo traj_info;
      if (IsLeadingObjectType(object_type) &&
          (MovingAlongPath(traj, path_sl, drive_passage, lc_left, &traj_info) ||
           CurrentlyNearTargetLanePath(traj, path_sl, drive_passage, lc_left,
                                       &traj_info))) {
        filtered_trajs.push_back(traj_info);
      }
    }

    std::sort(filtered_trajs.begin(), filtered_trajs.end(),
              [](const LeadingObjectTrajectoryInfo& traj1,
                 const LeadingObjectTrajectoryInfo& traj2) {
                return traj1.s < traj2.s;
              });

    // Group potential leading objects to groups. Separate groups according to
    // sufficient longitudinal gap.
    absl::flat_hash_set<std::string> leading_objs_set;
    const double min_gap = vehicle_geom.length() * 3.0;
    double previous_s = std::numeric_limits<double>::lowest();
    for (const auto& traj : filtered_trajs) {
      if (traj.s >= geom_max_accumulated_s) {
        continue;  // Ignore trajs far in front of sdc.
      }

      if (!leading_objs_set.empty()) {
        const double current_gap = traj.s - previous_s;
        if (current_gap >= min_gap) {
          groups_leading_objs.push_back(leading_objs_set);
          leading_objs_set.clear();
        }
      }
      leading_objs_set.insert(std::string(traj.traj_id));
      previous_s = traj.s;

      if (groups_leading_objs.size() == kMultiTrajMaxSize - 1) {
        break;
      }
    }
    if (!leading_objs_set.empty()) {
      groups_leading_objs.push_back(leading_objs_set);
    }
  } else {
    // Not executing lane change, do normal leading objects extraction.
    absl::flat_hash_set<std::string> leading_objs_set;
    for (const auto& [traj_id, leading_object] :
         constraint_manager.LeadingObjects()) {
      leading_objs_set.insert(std::string(traj_id));
    }
    groups_leading_objs.push_back(leading_objs_set);
  }

  // Construct search config according to groups. Add blocking static when
  // construct front objects.
  std::vector<InitializerSearchConfig> configs;
  if (lc_multiple_traj) {
    configs.reserve(groups_leading_objs.size() + 1);
    for (int i = 0; i <= groups_leading_objs.size(); ++i) {
      InitializerSearchConfig config;
      config.set_is_lane_change(is_lane_change);
      auto* leading_object_config = config.mutable_leading_object_config();
      if (groups_leading_objs.size() != 0) {
        if (i == 0) {
          // Follow the min accumulated s group. Rear is empty.
          GetLeadingObjectConfig(stalled_objects, &groups_leading_objs[0],
                                 nullptr, leading_object_config);
        } else if (i == groups_leading_objs.size()) {
          // Lead the max accumulated s group. Front is empty.
          GetLeadingObjectConfig(stalled_objects, nullptr,
                                 &groups_leading_objs[i - 1],
                                 leading_object_config);
        } else {
          // Track gap between groups.
          GetLeadingObjectConfig(stalled_objects, &groups_leading_objs[i],
                                 &groups_leading_objs[i - 1],
                                 leading_object_config);
        }
      }
      AddBlockingStaticObjToLeadingObjs(stalled_objects, blocking_static_obj,
                                        leading_object_config);
      configs.push_back(std::move(config));
    }
  } else {
    InitializerSearchConfig config;
    config.set_is_lane_change(is_lane_change);
    auto* leading_object_config = config.mutable_leading_object_config();
    if (groups_leading_objs.size() != 0) {
      if (is_lane_change) {
        // No multiple traj but lane changing.
        GetLeadingObjectConfig(stalled_objects, &groups_leading_objs[0],
                               nullptr, leading_object_config);
      } else {
        // Lane keeping, accept decision constraints.
        for (const auto& object : groups_leading_objs[0]) {
          leading_object_config->add_front(object);
        }
      }
    }
    AddBlockingStaticObjToLeadingObjs(stalled_objects, blocking_static_obj,
                                      leading_object_config);
    configs.push_back(std::move(config));
  }
  return configs;
}

int EvaluateMultiTrajs(const SpacetimeTrajectoryManager* st_traj_mgr,
                       const DrivePassage* passage,
                       const PlannerParamsProto& planner_params,
                       const std::vector<SingleTrajInfo>& multi_trajs,
                       const std::optional<ClearanceCheckOutput>* lc_clearance,
                       const VehicleGeometryParamsProto* vehicle_geom,
                       ThreadPool* thread_pool,
                       InitializerDebugProto* debug_proto) {
  SCOPED_QTRACE("EvaluateMultiTrajs");
  auto* selector_debug_proto = debug_proto->mutable_selector_debug_proto();
  if (multi_trajs.size() == 1) {
    VLOG(3) << "Only one trajectory. Return this choice anyway.";
    selector_debug_proto->set_single_choice_received(true);
    return 0;
  }
  selector_debug_proto->set_single_choice_received(false);

  // Collect leading object groups. multi_trajs should be in the ascending
  // order of accumulated_s of the group along the drive passage.

  std::vector<InitializerSelectorDebugProto::TrajectoryEvalInfo> evals;
  evals.reserve(multi_trajs.size());
  int considered_obj_size = 0;
  std::vector<std::string> all_leading_objs;
  for (const auto& traj : multi_trajs) {
    considered_obj_size += traj.lead_objs.size();
    const auto leading_obj_cost_or = GetDpLeadingObjectCost(traj);
    const auto final_lon_progress_or = GetFinalLongitudinalProgressCost(traj);
    const double leading_obj_cost = leading_obj_cost_or.value_or(0.0);
    const double final_lon_progress = final_lon_progress_or.value_or(0.0);

    InitializerSelectorDebugProto::TrajectoryEvalInfo eval_info;
    for (const auto& lead_obj : traj.lead_objs) {
      eval_info.add_leading_object(lead_obj);
    }
    eval_info.set_leading_object_cost(leading_obj_cost);
    eval_info.set_total_cost(traj.total_cost);
    eval_info.set_final_lon_progress_cost(final_lon_progress);
    eval_info.set_final_eval_cost(traj.total_cost - leading_obj_cost +
                                  final_lon_progress);
    evals.push_back(eval_info);
  }

  all_leading_objs.reserve(considered_obj_size);
  for (const auto& traj : multi_trajs) {
    for (const auto& id : traj.lead_objs) {
      all_leading_objs.push_back(id);
    }
  }

  VLOG(3) << "--------------- MultiTraj Evaluator -----------------";
  VLOG(3) << "considering object: " << absl::StrJoin(all_leading_objs, " ,");
  VLOG(3) << "evaluating " << multi_trajs.size() << " trajectories.";

  // Build frenet frame from drive passage's extend lane path.
  const auto& lane_path = passage->extend_lane_path();
  ASSIGN_OR_DIE(const auto target_frenet_frame,
                BuildKdTreeFrenetFrame(mapping::SampleLanePathPoints(
                    *passage->semantic_map_manager(), lane_path)));

  // For each lc trajectory, get the time when ego is considered on the
  // target lane. Evaluate the safe distance according to rss formulas and pick
  // the safest trajectory.
  ParallelFor(0, multi_trajs.size(), thread_pool, [&](int i) {
    // Parallel to calculate candidate motions and their cost for each
    // node on this layer.
    EvaluateSingleTrajectory(i, multi_trajs[i], all_leading_objs,
                             target_frenet_frame, planner_params, st_traj_mgr,
                             passage, lane_path, vehicle_geom, &evals[i]);
  });

  // Find the min cost trajectory idx.
  for (const auto& eval : evals) {
    auto* traj_eval_proto = selector_debug_proto->add_traj_eval();
    *traj_eval_proto = eval;
  }

  auto compare_func =
      [](const InitializerSelectorDebugProto::TrajectoryEvalInfo& eval_1,
         const InitializerSelectorDebugProto::TrajectoryEvalInfo& eval_2) {
        if (eval_1.leading_object_cost() < eval_2.leading_object_cost()) {
          // Consider leading object decision cost.
          return true;
        }
        if (eval_1.total_cost() < eval_2.total_cost()) {
          // If the trajectory satisfies leading object cost, consider post
          // evaluated trajectory cost result.
          return true;
        }
        return false;
      };
  auto choice = std::min_element(evals.begin(), evals.end(), compare_func);
  VLOG(3) << ">>>>>>>>>> Final Choice: traj_idx = " << choice - evals.begin();
  VLOG(3) << "-----------------------------------------------------";
  return choice - evals.begin();
}
}  // namespace qcraft::planner
