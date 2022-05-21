
#include "onboard/prediction/predictor/vehicle_lane_follow_predictor.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/global/trace.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/kinematic_predictor.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/trajectory_developer.h"
namespace qcraft {
namespace prediction {
namespace {

/*
 *   For associating agent's movement to lanes
 */
constexpr double kCTRARollOutHorizon = 1.5;     // s.
constexpr double kMaxHeadingDiff = M_PI / 3.0;  // rad.

/*
 *   Lane path generation related consts.
 */
constexpr double kForwardLane = 50.0;          // rad.
constexpr double kForwardExtendLength = 50.0;  // rad.
constexpr double kBackwardLength = 20.0;       // m.

// Sampling step of drive passage.
constexpr double kDrivePassageStepS = 2.0;

// We use kHistoryTime to consider the historical movement of the agent.
constexpr double kHistoryTime = 1.0;  // s.
constexpr double kDefaultHalfLaneWidth = 2.0;

/*
 *  Pure pursuit path generation related parameters.
 */
constexpr double kLateralSpeedLookAheadTime = 2.0;  // s.
constexpr double kLateralSpeedClamp = 2.5;          // m/s.
// Look ahead time for pure pursuit path generation.
constexpr double kPurePursuitLookAheadTime = 1.0;         // s.
constexpr double kNominalDtForPurePursuitPath = 0.3;      // s.
constexpr double kNominalHorizonForPurePursuitPath = 15;  // s.
constexpr double kCurvatureEpsilon = 1e-6;
// Look ahead time for curvature related acceleration heuristic.
constexpr double kAccLookAheadTime = 3.0;  // s.
// Minimal look ahead time for curvature related acceleration heuristic.
constexpr double kAccMinLookAheadTime = 1.0;  // s.
// Time to maintain heuristic acceleration.
constexpr double kMaintainAccTime = 1.5;         // s.
constexpr double kMaxLateralAcceleration = 2.5;  // m/s^2.

constexpr double kBikeLanePenalty = 0.1;
constexpr double kEmergencyLanePenalty = 0.2;

/*
 *  Final trajectory output related consts.
 */
constexpr int kMaxPrediction = 2;
constexpr double kKeepProb = 0.5;

const PiecewiseLinearFunction<double, double> kMinAccPLF({2.0, 5.0, 8.0},
                                                         {-4.0, -3.5, -2.5});
const PiecewiseLinearFunction<double, double> kMaxAccPLF({2.0, 5.0, 8.0},
                                                         {1.5, 0.5, 0.0});

// For curb cutoff.
constexpr double kVehNominalBrake = -2.0;  // m/s^2.

struct DrivePassageRankInfo {
  const planner::DrivePassage* dp = nullptr;
  double prob = 0.0;
};

/*
 * Cost is computed as the sum of predicted lateral offset cost and current
 * lateral offset cost. The offset cost is calculated as a sigmoid of vehicle's
 * lateral offset to the center line. if the lateral offset equals to the half
 * lane width, the cost is 0.5.
 */

double PredictedLateralOffsetCost(const ObjectHistorySpan& hist,
                                  const planner::DrivePassage& dp,
                                  double lateral_speed_clamp,
                                  double lateral_speed_pred_time) {
  const auto& last_obj = hist.back().val;
  const auto& pos = last_obj.pos();
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);
  if (!sl_or.ok()) {
    return 0.0;
  }
  const auto& sl = sl_or.value();
  double cur_lat_speed = LineFitLateralSpeedByHistory(hist, dp);

  cur_lat_speed =
      std::clamp(cur_lat_speed, -lateral_speed_clamp, lateral_speed_clamp);
  double target_l = 0.0;
  if (sl.l > 0) {
    target_l = std::max(0.0, sl.l + cur_lat_speed * lateral_speed_pred_time);
  } else {
    target_l = std::min(0.0, sl.l + cur_lat_speed * lateral_speed_pred_time);
  }

  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(sl.s);
  double min_l = -kDefaultHalfLaneWidth;
  double max_l = kDefaultHalfLaneWidth;
  if (bounds_or.ok()) {
    min_l = std::max(min_l, bounds_or->first);
    max_l = std::min(max_l, bounds_or->second);
  }
  double dis_to_bound = 0.0;
  if (target_l < 0) {
    dis_to_bound = target_l - min_l;
  } else {
    dis_to_bound = max_l - target_l;
  }

  const double cost = Sigmoid(dis_to_bound);
  return cost;
}

double CurrentLateralOffsetCost(const ObjectHistorySpan& hist,
                                const planner::DrivePassage& dp) {
  const auto& last_obj = hist.back().val;
  const auto& pos = last_obj.pos();
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);
  if (!sl_or.ok()) {
    return 0.0;
  }
  double cur_lateral_offset = std::numeric_limits<double>::max();
  cur_lateral_offset = sl_or->l;
  const auto ref_angle_or = dp.QueryTangentAngleAtS(sl_or->s);
  double angle_diff = 0.0;
  if (ref_angle_or.ok()) {
    angle_diff = NormalizeAngle(last_obj.heading() - ref_angle_or.value());
  }
  const double length = last_obj.bounding_box().length();
  cur_lateral_offset += (0.5 * length * std::sin(angle_diff));
  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(sl_or->s);
  double min_l = -kDefaultHalfLaneWidth;
  double max_l = kDefaultHalfLaneWidth;
  if (bounds_or.ok()) {
    min_l = std::max(min_l, bounds_or->first);
    max_l = std::min(max_l, bounds_or->second);
  }
  double dis_to_bound = 0.0;
  if (cur_lateral_offset < 0) {
    dis_to_bound = cur_lateral_offset - min_l;
  } else {
    dis_to_bound = max_l - cur_lateral_offset;
  }
  const double cost = Sigmoid(dis_to_bound);

  return cost;
}

double ComputeDrivePassageCost(const ObjectHistorySpan& hist,
                               const planner::DrivePassage& dp,
                               double lateral_speed_clamp,
                               double lateral_speed_pred_time) {
  const double future_cost = PredictedLateralOffsetCost(
      hist, dp, lateral_speed_clamp, lateral_speed_pred_time);
  const double cur_cost = CurrentLateralOffsetCost(hist, dp);
  double penalty = 1.0;
  for (const auto* lane_info : dp.lane_path().GetLanesInfo()) {
    if (lane_info->Type() == mapping::LaneProto::BICYCLE_ONLY) {
      penalty = std::min(penalty, kBikeLanePenalty);
    }
    if (lane_info->Type() == mapping::LaneProto::EMERGENCY) {
      penalty = std::min(penalty, kEmergencyLanePenalty);
    }
  }
  return penalty * std::max(future_cost, cur_cost);
}

planner::SpeedVector ComputeHeuristicSpeed(
    double cur_speed, double accel, const planner::DiscretizedPath& path) {
  // Approximatively compute decel by curvature limit.
  double dist = kAccLookAheadTime * cur_speed;
  double max_abs_curvature = 0.0;
  for (const auto& pt : path) {
    if (pt.s() < kAccMinLookAheadTime * cur_speed) {
      continue;
    }
    if (pt.s() > dist) {
      break;
    }
    max_abs_curvature = std::max(max_abs_curvature, std::fabs(pt.kappa()));
  }
  double des_speed = std::sqrt(kMaxLateralAcceleration /
                               std::max(max_abs_curvature, kCurvatureEpsilon));
  if (des_speed < cur_speed) {
    accel = std::min(accel, (des_speed - cur_speed) / kAccLookAheadTime);
  }
  double kMinAcc = kMinAccPLF(cur_speed);
  double kMaxAcc = kMaxAccPLF(cur_speed);
  // Get the final acceleration.
  accel = std::clamp(accel, kMinAcc, kMaxAcc);

  // Apply acceleration
  double t = 0.0;
  planner::SpeedVector speed_vec;
  speed_vec.reserve(
      static_cast<int>(kPredictionDuration / kPredictionTimeStep));
  planner::SpeedPoint sp0(t, /*double s=*/0.0, cur_speed, accel,
                          /*double jerk=*/0.0);
  speed_vec.push_back(std::move(sp0));
  t += kPredictionTimeStep;
  while (t < kPredictionDuration) {
    const auto& prev = speed_vec.back();
    planner::SpeedPoint sp1;
    sp1.set_t(t);
    if (t > kMaintainAccTime) {
      sp1.set_v(prev.v());
      sp1.set_s(prev.s() + prev.v() * kPredictionTimeStep);
      speed_vec.push_back(std::move(sp1));
    } else {
      const auto target_v =
          std::max(prev.v() + accel * kPredictionTimeStep, 0.0);
      sp1.set_v(target_v);
      sp1.set_s(prev.s() + 0.5 * (prev.v() + target_v) * kPredictionTimeStep);
      sp1.set_a((target_v - prev.v()) / kPredictionTimeStep);
      speed_vec.push_back(std::move(sp1));
    }

    t += kPredictionTimeStep;
  }
  return speed_vec;
}

std::vector<PredictedTrajectoryPoint> GeneratePredictedTrajectoryPoints(
    const ObjectHistorySpan& hist, const DrivePassageRankInfo& dp_rank,
    const ObjectPredictionScenario& scenario) {
  const auto& last_obj = hist.back().val;
  const auto& pos = last_obj.pos();
  const auto& dp = *dp_rank.dp;
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);
  if (!sl_or.ok()) {
    return std::vector<PredictedTrajectoryPoint>();
  }
  const auto& sl = sl_or.value();
  double cur_lat_speed = LineFitLateralSpeedByHistory(hist, dp);
  cur_lat_speed =
      std::clamp(cur_lat_speed, -kLateralSpeedClamp, kLateralSpeedClamp);
  double target_l = 0.0;
  if (sl.l > 0) {
    target_l = std::max(0.0, sl.l + cur_lat_speed * kLateralSpeedLookAheadTime);
  } else {
    target_l = std::min(0.0, sl.l + cur_lat_speed * kLateralSpeedLookAheadTime);
  }
  const auto bounds_or = dp.QueryNearestBoundaryLateralOffset(sl.s);
  double min_l = -kDefaultHalfLaneWidth;
  double max_l = kDefaultHalfLaneWidth;
  if (bounds_or.ok()) {
    min_l = std::max(min_l, bounds_or->first);
    max_l = std::min(max_l, bounds_or->second);
  }
  if (bounds_or.ok()) {
    target_l = std::clamp(target_l, min_l, max_l);
  }
  const double fitted_accel = LineFitAccelerationByHistory(hist);
  const Vec2d vec_accel(Vec2d(last_obj.object_proto().accel()));
  const auto tangent = Vec2d::UnitFromAngle(last_obj.heading());
  const double perception_accel = vec_accel.dot(tangent);
  const double accel = 0.5 * (fitted_accel + perception_accel);
  BicycleModelState obj_state{.x = pos.x(),
                              .y = pos.y(),
                              .v = last_obj.v(),
                              .heading = last_obj.heading(),
                              .acc = 0.0,
                              .front_wheel_angle = 0.0};
  const auto obj_box = BuildObjectBoundingBox(last_obj.object_proto());
  const auto half_object_length = obj_box.half_length();

  planner::DiscretizedPath path;
  if (scenario.intersection_status() == OIS_IN_INTERSECTION) {
    path = DevelopPurePursuitPath(
        obj_state, *dp_rank.dp, 0.0, kPurePursuitLookAheadTime,
        kNominalDtForPurePursuitPath, kNominalHorizonForPurePursuitPath,
        half_object_length, half_object_length, kVehCurvatureLimit);
  } else {
    path = DevelopPurePursuitPath(
        obj_state, *dp_rank.dp, target_l, kPurePursuitLookAheadTime,
        kNominalDtForPurePursuitPath, kNominalHorizonForPurePursuitPath,
        half_object_length, half_object_length, kVehCurvatureLimit);
  }
  double cur_speed = std::fabs(obj_state.v);
  const auto speed_vec = ComputeHeuristicSpeed(cur_speed, accel, path);
  return CombinePathAndSpeedForPredictedTrajectoryPoints(path, speed_vec);
}

std::vector<PredictedTrajectory> DevelopVehicleLaneFollowPrediction(
    const ObjectHistory* obj, const PredictionContext& context,
    const ObjectPredictionScenario& scenario,
    ObjectPredictionPriority priority) {
  const double last_t = obj->timestamp();
  const auto hist_or = obj->GetHistoryFrom(last_t - kHistoryTime);
  QCHECK_OK(hist_or.status());
  const auto& hist = hist_or.value();
  const auto& last_obj = hist.back().val;
  const auto& pos = last_obj.pos();
  const auto& semantic_map_mgr = *context.semantic_map_manager();
  const auto probe_traj_pts = DevelopCTRATrajectory(
      ObjectProtoToUniCycleState(last_obj.object_proto()), kPredictionTimeStep,
      kCTRARollOutHorizon, kEmergencyGuardHorizon, /*use_acc=*/true);
  auto associated_lane_ids =
      FindNearestLaneIdsByBBoxWithBoundaryDistLimitAndHeadingDiffLimit(
          semantic_map_mgr, hist.bounding_box(),
          /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);
  for (const auto& pt : probe_traj_pts) {
    const auto lane_id_or =
        FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
            semantic_map_mgr, pt.pos(), pt.theta(),
            /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);
    if (lane_id_or.has_value()) {
      associated_lane_ids.insert(*lane_id_or);
    }
  }
  const auto filtered_ids = FilterLanesByConsecutiveRelationship(
      semantic_map_mgr, associated_lane_ids);
  std::vector<mapping::LanePath> lane_paths;
  lane_paths.reserve(filtered_ids.size());
  for (const auto& id : filtered_ids) {
    const auto lps = SearchLanePath(pos, semantic_map_mgr, id, kForwardLane,
                                    /*is_reverse_driving=*/false);
    for (const auto& lp : lps) {
      // Extend the front part of lane path.
      auto extended_lp = ExtendMostStraightLanePath(
          lp, semantic_map_mgr, kBackwardLength, /*is_reversed=*/true);
      extended_lp = ExtendMostStraightLanePath(extended_lp, semantic_map_mgr,
                                               kForwardExtendLength,
                                               /*is_reversed=*/false);
      lane_paths.push_back(std::move(extended_lp));
    }
  }

  std::vector<planner::DrivePassage> dps;
  dps.reserve(lane_paths.size());
  for (const auto& lp : lane_paths) {
    dps.push_back(planner::BuildDrivePassageFromLanePath(
        semantic_map_mgr, lp, kDrivePassageStepS, /*avoid_loop=*/true));
  }
  std::vector<DrivePassageRankInfo> dp_ranks;
  dp_ranks.reserve(dps.size());
  for (const auto& dp : dps) {
    dp_ranks.push_back(
        {.dp = &dp,
         .prob = ComputeDrivePassageCost(hist, dp, kLateralSpeedClamp,
                                         kLateralSpeedLookAheadTime)});
  }

  std::sort(dp_ranks.begin(), dp_ranks.end(),
            [](const auto& a, const auto& b) { return a.prob > b.prob; });

  // Only keep dps with prob larger than 0.5 or keep the highest if all smaller
  // than 0.5;
  int erase_idx = 1;
  // Keep maximally kMaxPrediction predictions.
  for (; erase_idx < std::min<int>(dp_ranks.size(), kMaxPrediction);
       ++erase_idx) {
    if (dp_ranks[erase_idx].prob < kKeepProb) {
      break;
    }
  }
  if (erase_idx < dp_ranks.size()) {
    dp_ranks.resize(erase_idx);
  }
  std::vector<PredictedTrajectory> res;
  res.reserve(dp_ranks.size());
  double sum_prob = 0.0;
  for (const auto& dp_rank : dp_ranks) {
    auto traj_pts = GeneratePredictedTrajectoryPoints(hist, dp_rank, scenario);
    if (traj_pts.empty()) {
      continue;
    }
    res.push_back(PredictedTrajectory(
        /*probability=*/dp_rank.prob, priority,
        absl::StrCat("Veh lane follow:", dp_rank.dp->lane_path().DebugString()),
        PredictionType::PT_VEHICLE_LANE_FOLLOW, 0, std::move(traj_pts),
        /*is_reversed=*/false));
    sum_prob += dp_rank.prob;
  }
  if (res.empty()) {
    auto ctra_traj_pts = DevelopCTRATrajectory(
        ObjectProtoToUniCycleState(last_obj.object_proto()),
        kPredictionTimeStep, kEmergencyGuardHorizon, kSafeHorizon,
        /*use_acc=*/true);
    return {PredictedTrajectory(
        /*probability=*/1.0, priority, "Veh lane follow: CTRA (no lane path)",
        PredictionType::PT_VEHICLE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }
  QCHECK_GT(sum_prob, 0.0);
  for (auto& traj : res) {
    traj.set_probability(traj.probability() / sum_prob);
  }
  return res;
}
}  // namespace
std::vector<PredictedTrajectory> MakeVehicleLaneFollowPrediction(
    const ObjectHistory* obj, const PredictionContext& context,
    const ObjectPredictionScenario& scenario,
    ObjectPredictionPriority priority) {
  SCOPED_QTRACE("MakeVehicleLaneFollowPrediction");

  auto trajs =
      DevelopVehicleLaneFollowPrediction(obj, context, scenario, priority);
  const auto hist_or = obj->GetHistory();
  QCHECK_OK(hist_or.status());
  const auto& hist = hist_or.value();
  const double object_length = hist.bounding_box().length();
  for (auto& traj : trajs) {
    const auto curb_collision_infos = FindTrajCurbCollisionIndex(
        traj.points(), *context.semantic_map_manager(), object_length,
        qcraft::ObjectType::OT_VEHICLE);
    CutoffTrajByCurbCollisionIndexes(curb_collision_infos, kVehNominalBrake,
                                     kEmergencyGuardHorizon, kSafeHorizon,
                                     &traj);
  }
  return trajs;
}
}  // namespace prediction
}  // namespace qcraft
