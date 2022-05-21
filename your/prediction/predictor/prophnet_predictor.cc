#include "onboard/prediction/predictor/prophnet_predictor.h"

#include <algorithm>
#include <queue>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/math/line_fitter.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/kinematic_predictor.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(extend_prophnet_trajectory_by_pure_pursuit, true,
            "Extend prophnet trajectory by pure pursuit.");
DEFINE_bool(extend_prophnet_trajectory_by_pole_placement, false,
            "Extend prophnet trajectory by pole placement.");
namespace qcraft {
namespace prediction {

namespace {
constexpr double kStationarySpeedThreshold = 0.2;          // m/s.
constexpr double kNominalBrakeForTrajCutoffAtCurb = -2.0;  // m/s^2
constexpr double kProbThreshold = 0.2;
constexpr double kMinMoveDis = 1.5;  // min move meters.
constexpr int kFitterDegree = 5;
constexpr double kPredDisToIntersection = 200.0;
constexpr double kDistEpsilon = 1.0;
constexpr double kVehicleKappaCutOff = 0.3;  // m^-1.

constexpr double kMinHistLengthForOnRoadObj = 0.3;     // s.
constexpr double kDistanceEpsilon = 0.1;               // m.
constexpr double kExtendAngleDiffLimit = M_PI / 12.0;  // rad.
constexpr double kExtendForwardLen = 100.0;            // m.
constexpr double kPathStepS = 2.0;                     // m.
constexpr double kPreviewT = 1.5;                      // s.
// Feature extraction related consts.
constexpr double kHighSpeedThreshold = 15.0;   // m/s.
constexpr double kMediumSpeedThreshold = 6.0;  // m/s.
constexpr double kEvalSpeedTime = 3.0;         // s.
constexpr double kAccForLabel = 1.0;           // m/s^2.
constexpr double kTimeEpsilon = 0.1;           // s.
constexpr double kInitIndexWeight = 10.0;      // s.

struct TrajFitter {
  PolynomialFitter x_fitter;
  PolynomialFitter y_fitter;
};

TrajFitter GetTrajFitter(absl::Span<const Vec2d> traj_points,
                         absl::Span<const double> vec_t) {
  int size = traj_points.size();
  std::vector<Vec2d> x_series, y_series;
  std::vector<double> x_weights, y_weights;
  x_series.reserve(size);
  y_series.reserve(size);
  for (int i = 0; i < size; i++) {
    if (i == 0) {
      x_weights.push_back(kInitIndexWeight);
      y_weights.push_back(kInitIndexWeight);
    } else {
      x_weights.push_back(1.0);
      y_weights.push_back(1.0);
    }
    x_series.emplace_back(vec_t[i], traj_points[i].x());
    y_series.emplace_back(vec_t[i], traj_points[i].y());
  }
  PolynomialFitter x_fitter(kFitterDegree, x_series, x_weights);
  PolynomialFitter y_fitter(kFitterDegree, y_series, y_weights);
  x_fitter.FitData(LS_SOLVER::QR);
  y_fitter.FitData(LS_SOLVER::QR);
  return {x_fitter, y_fitter};
}

ObjectProbTrajs FilterTrajsByProb(const ObjectProbTrajs &prob_trajs) {
  ObjectProbTrajs filtered_trajs;
  filtered_trajs.reserve(prob_trajs.size());
  // Trajectory probability is ordered from large to small.
  filtered_trajs.push_back(prob_trajs.front());
  int valid_traj_idx = 1;
  double sum_prob = prob_trajs.front().first;
  while (valid_traj_idx < prob_trajs.size()) {
    if (prob_trajs[valid_traj_idx].first < kProbThreshold) {
      break;
    }
    filtered_trajs.push_back(prob_trajs[valid_traj_idx]);
    sum_prob += prob_trajs[valid_traj_idx].first;
    ++valid_traj_idx;
  }
  QCHECK_GT(sum_prob, 0.0);
  for (auto &prob_traj : filtered_trajs) {
    prob_traj.first /= sum_prob;
  }
  return filtered_trajs;
}

bool IsTrajTooShort(absl::Span<const PredictedTrajectoryPoint> traj) {
  if (traj.front().pos().DistanceSquareTo(traj.back().pos()) <
      Sqr(kMinMoveDis)) {
    return true;
  }
  return false;
}

void CutOffTrajectoryPointsByCurvature(
    absl::Span<PredictedTrajectoryPoint> traj_pts) {
  for (int i = 1; i < traj_pts.size(); ++i) {
    if (std::fabs(traj_pts[i].kappa()) > kVehicleKappaCutOff) {
      for (int j = i; j < traj_pts.size(); ++j) {
        traj_pts[j] = traj_pts[i - 1];
        traj_pts[j].set_v(0.0);
      }
      break;
    }
  }
}

void ExtendFinalStoppedTrajectoryPoints(
    std::vector<PredictedTrajectoryPoint> *traj_pts) {
  if (traj_pts->empty() || traj_pts->back().v() > kStationarySpeedThreshold) {
    return;
  }
  auto pt = traj_pts->back();
  pt.set_v(0.0);
  for (int i = traj_pts->size() - 1;
       i < static_cast<int>(kPredictionDuration / kPredictionTimeStep); ++i) {
    pt.set_t(i * kPredictionTimeStep);
    traj_pts->push_back(pt);
  }
}

void TryExtendCruisingTrajectoryPoints(
    const SemanticMapManager &semantic_map_mgr, double obj_len,
    double obj_width, std::vector<PredictedTrajectoryPoint> *traj_pts) {
  if (traj_pts->empty()) {
    return;
  }
  const auto &pt = traj_pts->back();
  const auto last_t = pt.t();
  if (last_t >= kPredictionDuration) return;
  const auto &pos = pt.pos();
  // Check if target point is inside the intersection.
  const auto *intersection_ptr =
      semantic_map_mgr.GetNearestIntersectionInfoAtLevel(
          semantic_map_mgr.GetLevel(), pos);
  if (intersection_ptr != nullptr) {
    const double cur_distance =
        intersection_ptr->polygon_smooth.DistanceTo(pos);
    // Do not extend if the last point is inside junction.
    if (cur_distance < kDistanceEpsilon) {
      return;
    }
  }
  // Find the closest lane id with a strict condition.
  const auto nearest_lane_id_or =
      FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
          semantic_map_mgr, pos, pt.theta(), /*boundary_distance_limit=*/0.0,
          kExtendAngleDiffLimit);
  // Do not extend if the nearest lane id is not well aligned with the closest
  // lane.
  if (nearest_lane_id_or == std::nullopt) {
    return;
  }
  // Extend lane to a lane path by forward search.
  const auto lane_path = SearchMostStraightLanePath(
      pos, semantic_map_mgr, *nearest_lane_id_or, kExtendForwardLen,
      /*is_reverse_driving=*/false);
  // Build drive passage.
  const auto dp = planner::BuildDrivePassageFromLanePath(semantic_map_mgr,
                                                         lane_path, kPathStepS,
                                                         /*avoid_loop=*/true);
  std::vector<PredictedTrajectoryPoint> extended_pts;
  BicycleModelState state{.x = pos.x(),
                          .y = pos.y(),
                          .v = pt.v(),
                          .heading = pt.theta(),
                          .acc = 0.0,
                          .front_wheel_angle = 0.0};
  if (FLAGS_extend_prophnet_trajectory_by_pure_pursuit) {
    const auto target_sl_or = dp.QueryFrenetCoordinateAt(pos);
    std::optional<double> target_l_or;
    if (target_sl_or.ok()) {
      target_l_or = target_sl_or->l;
      const auto bounds_or =
          dp.QueryNearestBoundaryLateralOffset(target_sl_or->s);
      const double half_width = 0.5 * obj_width;
      if (bounds_or.ok()) {
        target_l_or = std::clamp(*target_l_or, bounds_or->first + half_width,
                                 bounds_or->second - half_width);
      }
    }
    if (target_l_or.has_value()) {
      const auto &target_l = target_l_or.value();
      // Extend trajectory by pure pursuit algorithm.
      const auto discretized_path = DevelopPurePursuitPath(
          state, dp, target_l, kPreviewT, kPredictionTimeStep,
          kPredictionDuration - last_t, 0.5 * obj_len, 0.5 * obj_len,
          kVehicleKappaCutOff);
      const auto speed_vec = DevelopConstVelocitySpeedProfile(
          pt.v(), kPredictionTimeStep, kPredictionDuration - last_t);
      extended_pts = CombinePathAndSpeedForPredictedTrajectoryPoints(
          discretized_path, speed_vec);
    } else if (FLAGS_extend_prophnet_trajectory_by_pole_placement) {
      auto extended_pts_or = DevelopConstVelocityPolePlacementTrajectory(
          state, dp, kPredictionTimeStep, kSafeHorizon, obj_len);
      if (extended_pts_or.ok()) {
        extended_pts = std::move(extended_pts_or).value();
      }
    }
  }

  double cur_t = pt.t();
  double cur_s = pt.s();
  for (int i = 0; i < extended_pts.size(); ++i) {
    extended_pts[i].set_t(cur_t + i * kPredictionTimeStep);
    extended_pts[i].set_s(cur_s + extended_pts[i].s());
    traj_pts->push_back(std::move(extended_pts[i]));
  }
}

std::vector<ObjectIDType> GetPredictObjectIds(
    const ObjectIDType &av_object_id,
    absl::Span<const ObjectHistory *const> objs) {
  std::vector<ObjectIDType> predict_ids;
  // Add av_object as the last to predict.
  predict_ids.reserve(objs.size() + 1);
  for (const auto &obj : objs) {
    predict_ids.push_back(obj->id());
  }
  predict_ids.push_back(av_object_id);
  return predict_ids;
}

ResampledObjectsHistory GetResampledHistory(
    const ObjectHistorySpan &av_object,
    absl::Span<const ObjectHistory *const> objs, double current_ts) {
  return GetResampledHistory(av_object, objs, current_ts,
                             prophnet::kHistoryNum);
}

std::vector<PredictedTrajectoryPoint> GenerateTrajectoryPoints(
    const TrajFitter &traj_fitter, int points_size, double start_t) {
  std::vector<PredictedTrajectoryPoint> traj_pts;
  traj_pts.reserve(points_size);
  double s = 0.0;
  Vec2d prev_pos;
  for (int i = 0; i < points_size; ++i) {
    const double point_t = static_cast<double>(i) * kPredictionTimeStep;
    const double t = start_t + point_t;
    const double x = traj_fitter.x_fitter.Evaluate(t);
    const double dx = traj_fitter.x_fitter.EvaluateDerivative(t, 1);
    const double d2x = traj_fitter.x_fitter.EvaluateDerivative(t, 2);
    const double y = traj_fitter.y_fitter.Evaluate(t);
    const double dy = traj_fitter.y_fitter.EvaluateDerivative(t, 1);
    const double d2y = traj_fitter.y_fitter.EvaluateDerivative(t, 2);
    PredictedTrajectoryPoint point;
    point.set_t(point_t);
    Vec2d cur_pos(x, y);
    point.set_pos(cur_pos);
    Vec2d d(dx, dy);
    Vec2d d2(d2x, d2y);
    point.set_v(d.norm());
    if (d2.dot(d) > 0) {
      point.set_a(d2.norm());
    } else {
      point.set_a(-d2.norm());
    }
    point.set_theta(d.Angle());
    point.set_kappa((dx * d2y - dy * d2x) / std::pow(dx * dx + dy * dy, 1.5));
    if (i > 0) {
      s += (cur_pos - prev_pos).norm();
    }
    prev_pos = cur_pos;
    point.set_s(s);
    traj_pts.push_back(std::move(point));
  }
  return traj_pts;
}

std::vector<PredictedTrajectory> PostProcessingTrajectories(
    const ObjectHistorySpan &obj, const ObjectProbTrajs &prob_trajs,
    ObjectPredictionPriority prior, const SemanticMapManager &semantic_map_mgr,
    const TrafficLightManager::TLStateHashMap &tl_state_map,
    double predict_start_time) {
  const auto &object_proto = obj.back().val.object_proto();
  double t_diff = object_proto.timestamp() - predict_start_time;
  int traj_index = -1;
  const auto filtered_trajs = FilterTrajsByProb(prob_trajs);
  std::vector<PredictedTrajectory> pred_trajs;
  pred_trajs.reserve(filtered_trajs.size());
  for (const auto &prob_traj : filtered_trajs) {
    traj_index++;
    if (object_proto.moving_state() == ObjectProto_MovingStateProto_MS_STATIC) {
      auto traj_pts =
          DevelopStaticTrajectory(ObjectProtoToUniCycleState(object_proto),
                                  kPredictionTimeStep, kPredictionDuration);
      return {PredictedTrajectory(
          /*probability=*/1.0, prior, "Stationary Prediction from ProphNet",
          PredictionType::PT_STATIONARY,
          /*index=*/0, std::move(traj_pts),
          /*is_reversed=*/false)};
    }
    std::vector<Vec2d> traj_points;
    std::vector<double> vec_t;
    traj_points.reserve(FloorToInt(kPredictionDuration / kPredictionTimeStep));
    // Here assuming that the current pos is the first predicted point.
    traj_points.emplace_back(object_proto.pos());
    vec_t.push_back(t_diff);
    for (int i = 0; i < prob_traj.second.size(); ++i) {
      traj_points.push_back(prob_traj.second[i]);
      vec_t.push_back(i * kPredictionTimeStep);
    }
    // Fit trajectory points by polynomial fitting.
    const auto traj_fitter = GetTrajFitter(traj_points, vec_t);
    auto traj_pts =
        GenerateTrajectoryPoints(traj_fitter, traj_points.size(), t_diff);
    // Cut off trajectory if curvature is too large
    if (object_proto.type() == OT_VEHICLE) {
      CutOffTrajectoryPointsByCurvature(absl::MakeSpan(traj_pts));
    }
    if (traj_pts.back().v() < kStationarySpeedThreshold) {
      // For stopping trajectory, extend to 10s by repeating stationary
      // trajectory points
      ExtendFinalStoppedTrajectoryPoints(&traj_pts);
    } else {
      // Try extend trajectory to 10s (only extend if end trajectory point is
      // out of junction).
      const auto box = BuildObjectBoundingBox(object_proto);
      TryExtendCruisingTrajectoryPoints(semantic_map_mgr, box.length(),
                                        box.width(), &traj_pts);
    }
    std::string annotation = "Predicted by Prophnet model";
    const bool is_traj_too_short = IsTrajTooShort(traj_pts);
    if (is_traj_too_short && object_proto.type() == OT_VEHICLE) {
      traj_pts = DevelopCTRATrajectory(
          ObjectProtoToUniCycleState(object_proto), kPredictionTimeStep,
          kEmergencyGuardHorizon, kComfortableHorizon,
          /*is_reversed=*/false);
      annotation += ", use CTRA 8s";
    }
    PredictedTrajectory pred_traj(prob_traj.first, prior, annotation,
                                  PredictionType::PT_PROPHNET, traj_index,
                                  traj_pts, false);
    // Cut off by curb
    const double object_length = BuildObjectBoundingBox(object_proto).length();
    const auto curb_collision_infos =
        FindTrajCurbCollisionIndex(pred_traj.points(), semantic_map_mgr,
                                   object_length, object_proto.type());

    double min_horizon = kEmergencyGuardHorizon;
    switch (object_proto.type()) {
      case OT_VEHICLE:
      case OT_CYCLIST:
      case OT_MOTORCYCLIST:
        min_horizon = kEmergencyGuardHorizon;
        break;
      case OT_PEDESTRIAN:
        min_horizon = kSafeHorizon;
        break;
      case OT_UNKNOWN_STATIC:
      case OT_UNKNOWN_MOVABLE:
      case OT_FOD:
      case OT_VEGETATION:
      case OT_BARRIER:
      case OT_CONE:
      case OT_WARNING_TRIANGLE:
        min_horizon = kEmergencyGuardHorizon;
        break;
    }
    CutoffTrajByCurbCollisionIndexes(curb_collision_infos,
                                     kNominalBrakeForTrajCutoffAtCurb,
                                     min_horizon, kSafeHorizon, &pred_traj);
    // Cut off by traffic lights.
    CutoffTrajByTrafficLightStatus(tl_state_map, semantic_map_mgr, &pred_traj);
    pred_trajs.push_back(pred_traj);
  }

  return pred_trajs;
}

std::vector<const ObjectHistory *>
ScreenOutProphnetPredictObjectsForOnePriority(
    const Box2d &ego_box,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    int max_objects_num) {
  if (objs_to_predict.size() <= max_objects_num) {
    return std::vector<const ObjectHistory *>(objs_to_predict.begin(),
                                              objs_to_predict.end());
  }
  std::vector<const ObjectHistory *> top_objs, medium_objs, low_objs;
  top_objs.reserve(max_objects_num);
  medium_objs.reserve(max_objects_num);
  low_objs.reserve(max_objects_num);
  const absl::flat_hash_set<ObjectType> top_priority_types = {
      OT_VEHICLE, OT_CYCLIST, OT_MOTORCYCLIST};
  const absl::flat_hash_set<ObjectType> medium_priority_types = {OT_PEDESTRIAN};
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    if (top_priority_types.contains(objs_to_predict[i]->type())) {
      top_objs.push_back(objs_to_predict[i]);
    } else if (medium_priority_types.contains(objs_to_predict[i]->type())) {
      medium_objs.push_back(objs_to_predict[i]);
    } else {
      low_objs.push_back(objs_to_predict[i]);
    }
  }
  // 1. Put vehicles and cyclists into the screened objs.
  std::vector<const ObjectHistory *> candidates =
      ScreenPredictObjectsByDistance(ego_box, top_objs, max_objects_num);
  if (candidates.size() == max_objects_num) {
    return candidates;
  }
  // 2. If we still have some space, put pedestrians inside.
  QCHECK_GT(max_objects_num - candidates.size(), 0);
  auto medium_candidates = ScreenPredictObjectsByDistance(
      ego_box, medium_objs, max_objects_num - candidates.size());

  std::move(medium_candidates.begin(), medium_candidates.end(),
            std::back_inserter(candidates));
  if (candidates.size() == max_objects_num) {
    return candidates;
  }

  // 3. If we still have some space, put other stuff inside.
  QCHECK_GT(max_objects_num - candidates.size(), 0);
  auto low_candidates = ScreenPredictObjectsByDistance(
      ego_box, low_objs, max_objects_num - candidates.size());

  std::move(low_candidates.begin(), low_candidates.end(),
            std::back_inserter(candidates));
  return candidates;
}
std::vector<const ObjectHistory *> ScreenOutProphnetPredictObjects(
    const Box2d &ego_box,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionPriority> &priorities,
    int max_objects_num) {
  const Box2d obj_region_box = GetRegionBox(
      ego_box.center(), ego_box.heading(), kProphnetObjectPredictionRegionFront,
      kProphnetObjectPredictionRegionBehind,
      kProphnetObjectPredictionHalfWidth);

  const std::vector<const ObjectHistory *> objs_in_box =
      GetObjectsInBox2d(objs_to_predict, obj_region_box);
  if (objs_in_box.size() <= max_objects_num) {
    return std::vector<const ObjectHistory *>(objs_in_box.begin(),
                                              objs_in_box.end());
  }
  std::vector<const ObjectHistory *> high_priority_objs, low_priority_objs;
  high_priority_objs.reserve(objs_in_box.size());
  low_priority_objs.reserve(objs_in_box.size());
  for (const auto *obj : objs_in_box) {
    const auto *priority = FindOrNull(priorities, obj->id());
    QCHECK_NOTNULL(priority);
    if (*priority == OPP_P0 || *priority == OPP_P1 || *priority == OPP_P2) {
      high_priority_objs.push_back(obj);
    } else {
      low_priority_objs.push_back(obj);
    }
  }
  auto pred_objs = ScreenOutProphnetPredictObjectsForOnePriority(
      ego_box, high_priority_objs, max_objects_num);
  QCHECK_LE(pred_objs.size(), max_objects_num);
  if (pred_objs.size() == max_objects_num) {
    return pred_objs;
  }
  auto pred_objs2 = ScreenOutProphnetPredictObjectsForOnePriority(
      ego_box, low_priority_objs, max_objects_num - pred_objs.size());
  std::move(pred_objs2.begin(), pred_objs2.end(),
            std::back_inserter(pred_objs));
  QCHECK_LE(pred_objs.size(), max_objects_num);
  return pred_objs;
}

ObjectsPredTrajsMap PostProcessProphnetResults(
    const ProphnetPredictionInput &prophnet_prediction_input,
    const ObjectsProbTrajs &pred_results, double predict_start_time) {
  ObjectsPredTrajsMap objects_pred_trajs_map;
  for (int i = 0; i < prophnet_prediction_input.objs.size(); ++i) {
    // Filter prophnet predicted trajs by scenarios.
    double hist_start_time =
        prophnet_prediction_input.objs[i]->GetHistory().value().front().time;
    double hist_len = predict_start_time - hist_start_time;
    const bool is_not_prophnet_obj =
        (hist_len < kMinHistLengthForOnRoadObj &&
         prophnet_prediction_input.scenarios[i]
                 .abs_dist_to_nearest_intersection() > kDistEpsilon);
    const bool is_prophnet_obj =
        (prophnet_prediction_input.scenarios[i]
                 .abs_dist_to_nearest_intersection() <=
             kPredDisToIntersection ||
         prophnet_prediction_input.objs[i]->type() == OT_CYCLIST ||
         prophnet_prediction_input.objs[i]->type() == OT_MOTORCYCLIST ||
         prophnet_prediction_input.objs[i]->type() == OT_PEDESTRIAN) &&
        (!is_not_prophnet_obj);
    if (!is_prophnet_obj) {
      continue;
    }

    objects_pred_trajs_map[prophnet_prediction_input.objs[i]->id()] =
        PostProcessingTrajectories(
            prophnet_prediction_input.objs[i]->GetHistory().value(),
            *FindOrNull(pred_results, prophnet_prediction_input.objs[i]->id()),
            prophnet_prediction_input.priors[i],
            *prophnet_prediction_input.semantic_map_mgr,
            *prophnet_prediction_input.inferred_tl_state_map,
            predict_start_time);
  }
  return objects_pred_trajs_map;
}

ProphnetDumpedFeatureProto::LabelsProto GenerateLabels(
    const PredictionContext &context,
    absl::Span<const ObjectIDType> predicted_ids,
    const std::map<ObjectIDType, const ObjectPredictionProto *> &log_pred_map) {
  ProphnetDumpedFeatureProto::LabelsProto labels;
  // Number of objects.
  labels.set_num_objects(predicted_ids.size() - 1);
  int num_vehs = 0;
  int num_cycs = 0;
  int num_peds = 0;

  // Motion of objects.
  int stationary_objs = 0;
  int braking_objs = 0;
  int acc_objs = 0;
  int high_speed_objs = 0;
  int medium_speed_objs = 0;
  int low_speed_objs = 0;
  const auto &objs_history = context.object_history_manager();
  for (const auto &id : predicted_ids) {
    if (id == kAvObjectId) {
      continue;
    }
    const auto *hist = objs_history.FindOrNull(id);
    QCHECK_NOTNULL(hist);
    const auto hist_span_or = hist->GetHistory();
    const auto &obj_hist = hist_span_or.value();
    switch (obj_hist.type()) {
      case OT_VEHICLE:
        ++num_vehs;
        break;
      case OT_MOTORCYCLIST:
      case OT_CYCLIST:
        ++num_cycs;
        break;
      case OT_PEDESTRIAN:
        ++num_peds;
        break;
      case OT_UNKNOWN_STATIC:
      case OT_FOD:
      case OT_UNKNOWN_MOVABLE:
      case OT_VEGETATION:
      case OT_BARRIER:
      case OT_CONE:
      case OT_WARNING_TRIANGLE:
        break;
    }
    const auto &obj = obj_hist.back().val;
    const double v = obj.v();
    // Count object speed
    if (v > kHighSpeedThreshold) {
      ++high_speed_objs;
    } else if (v > kMediumSpeedThreshold) {
      ++medium_speed_objs;
    } else {
      ++low_speed_objs;
    }
    // Count object tendancy
    const auto *gt = FindOrNull(log_pred_map, id);
    QCHECK_NOTNULL(gt);
    const auto &future_traj = (*gt)->trajectories(0);
    const int target_idx =
        std::min(static_cast<int>(kEvalSpeedTime / kPredictionTimeStep),
                 future_traj.points_size() - 1);
    const double future_v = future_traj.points(target_idx).v();
    const double acc =
        (future_v - v) / (target_idx * kPredictionTimeStep + kTimeEpsilon);
    if (acc > kAccForLabel) {
      ++acc_objs;
    } else if (acc < -kAccForLabel) {
      ++braking_objs;
    }
    if (obj.IsStationary()) {
      ++stationary_objs;
    }
  }
  labels.set_num_vehicles(num_vehs);
  labels.set_num_cyclists(num_cycs);
  labels.set_num_pedestrians(num_peds);
  labels.set_num_stationary(stationary_objs);
  labels.set_num_braking(braking_objs);
  labels.set_num_accelerating(acc_objs);
  labels.set_num_high_speed(high_speed_objs);
  labels.set_num_medium_speed(medium_speed_objs);
  labels.set_num_low_speed(low_speed_objs);

  return labels;
}

}  // namespace

ObjectsPredTrajsMap MakeProphnetPrediction(
    const ProphnetPredictionInput &prophnet_prediction_input,
    const ProphnetPredictorInput &prophnet_predictor_input,
    const ProphNetPredictor &prophnet_predictor) {
  FUNC_QTRACE();
  const auto &pred_results = prophnet_predictor.PredictForObjects(
      prophnet_predictor_input.input_features,
      prophnet_predictor_input.predict_ids);
  {
    SCOPED_QTRACE("PostprocessProphnetResults");
    return PostProcessProphnetResults(
        prophnet_prediction_input, pred_results.value(),
        prophnet_predictor_input.predict_start_time);
  }
}

ProphnetPredictorInput PrepareProphnetPredictorInput(
    const ProphnetPredictionInput &prophnet_prediction_input) {
  SCOPED_QTRACE_ARG1("PrepareProphnetPredictorInput",
                     "object_num:", prophnet_prediction_input.objs.size());
  const auto predict_ids = GetPredictObjectIds(
      prophnet_prediction_input.av_obj.id(), prophnet_prediction_input.objs);
  const double current_ts = GetCurrentTimeStamp(prophnet_prediction_input.objs);
  const auto objects_history =
      GetResampledHistory(prophnet_prediction_input.av_obj,
                          prophnet_prediction_input.objs, current_ts);
  const auto input_features = ExtractProphnetFeature(
      objects_history, *prophnet_prediction_input.original_tl_state_map,
      prophnet_prediction_input.semantic_map_mgr);
  return {.input_features = std::move(input_features),
          .predict_ids = std::move(predict_ids),
          .predict_start_time = current_ts};
}

ProphnetPredictionInput PrepareProphnetPredictionInput(
    const PredictionContext &prediction_context,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario> &object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriority> &object_priorities,
    int max_objects_num) {
  SCOPED_QTRACE("PrepareProphnetPredictionInput");
  const auto av_object_or =
      prediction_context.av_context().GetAvObjectHistory().GetHistory();
  QCHECK_OK(av_object_or.status());
  const auto screened_objs = ScreenOutProphnetPredictObjects(
      av_object_or->back().val.bounding_box(), objs_to_predict,
      object_priorities, max_objects_num);
  std::vector<ObjectPredictionScenario> screened_object_scenarios;
  std::vector<ObjectPredictionPriority> screened_object_priorities;
  screened_object_scenarios.reserve(screened_objs.size());
  screened_object_priorities.reserve(screened_objs.size());
  for (const auto &obj : screened_objs) {
    screened_object_scenarios.push_back(object_scenarios.at(obj->id()));
    screened_object_priorities.push_back(object_priorities.at(obj->id()));
  }

  return {
      .av_obj = std::move(av_object_or.value()),
      .objs = std::move(screened_objs),
      .priors = std::move(screened_object_priorities),
      .scenarios = std::move(screened_object_scenarios),
      .semantic_map_mgr = prediction_context.semantic_map_manager(),
      .inferred_tl_state_map =
          &prediction_context.traffic_light_manager().GetInferedTlStateMap(),
      .original_tl_state_map =
          &prediction_context.traffic_light_manager().GetOriginalTlStateMap()};
}

ProphnetDumpedFeatureProto ToProphnetDumpedFeatureProto(
    const PredictionContext &prediction_context,
    const ProphnetPredictorInput &prophnet_predictor_input,
    const std::map<ObjectIDType, ObjectPredictionScenario> &object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriority> &object_priorities,
    const std::map<ObjectIDType, const ObjectPredictionProto *> &log_pred_map,
    const TrajectoryProto &av_traj) {
  ProphnetDumpedFeatureProto features_proto;
  auto *mutable_objects_feature = features_proto.mutable_objects_feature();
  const auto &actors_feature =
      prophnet_predictor_input.input_features.actors_feature;
  *mutable_objects_feature->mutable_ego_pos() = {actors_feature.ego_pos.begin(),
                                                 actors_feature.ego_pos.end()};
  *mutable_objects_feature->mutable_rot_mat() = {actors_feature.rot_mat.begin(),
                                                 actors_feature.rot_mat.end()};
  *mutable_objects_feature->mutable_trajs() = {actors_feature.trajs.begin(),
                                               actors_feature.trajs.end()};
  *mutable_objects_feature->mutable_speeds() = {actors_feature.speeds.begin(),
                                                actors_feature.speeds.end()};
  *mutable_objects_feature->mutable_headings() = {
      actors_feature.headings.begin(), actors_feature.headings.end()};
  *mutable_objects_feature->mutable_types() = {actors_feature.types.begin(),
                                               actors_feature.types.end()};
  *mutable_objects_feature->mutable_cur_poses() = {
      actors_feature.cur_poses.begin(), actors_feature.cur_poses.end()};

  *mutable_objects_feature->mutable_shapes() = {actors_feature.shapes.begin(),
                                                actors_feature.shapes.end()};
  *mutable_objects_feature->mutable_max_curvatures() = {
      actors_feature.max_ks.begin(), actors_feature.max_ks.end()};
  *mutable_objects_feature->mutable_max_lateral_accel() = {
      actors_feature.max_lat_accs.begin(), actors_feature.max_lat_accs.end()};
  *mutable_objects_feature->mutable_dist_to_racs() = {
      actors_feature.dist_to_racs.begin(), actors_feature.dist_to_racs.end()};
  *mutable_objects_feature->mutable_is_statics() = {
      actors_feature.is_statics.begin(), actors_feature.is_statics.end()};
  auto *mutable_lanes_feature = features_proto.mutable_lanes_feature();
  const auto &lanes_feature =
      prophnet_predictor_input.input_features.lanes_feature;
  *mutable_lanes_feature->mutable_lane_centers() = {
      lanes_feature.lane_centers.begin(), lanes_feature.lane_centers.end()};
  *mutable_lanes_feature->mutable_lane_segments() = {
      lanes_feature.lane_segments.begin(), lanes_feature.lane_segments.end()};
  *mutable_lanes_feature->mutable_lane_controls() = {
      lanes_feature.lane_controls.begin(), lanes_feature.lane_controls.end()};
  *mutable_lanes_feature->mutable_lane_lights() = {
      lanes_feature.lane_lights.begin(), lanes_feature.lane_lights.end()};
  features_proto.mutable_gt_trajs()->Reserve(
      prophnet_predictor_input.predict_ids.size());
  // last id is av.
  for (int i = 0; i < prophnet_predictor_input.predict_ids.size() - 1; ++i) {
    const auto &id = prophnet_predictor_input.predict_ids[i];
    auto *gt_traj = features_proto.mutable_gt_trajs()->Add();
    gt_traj->set_object_id(id);
    std::vector<PredictedTrajectoryPointProto> raw_traj;
    for (const auto &pt : log_pred_map.at(id)->trajectories().Get(0).points()) {
      raw_traj.push_back(pt);
    }
    gt_traj->set_av_relation(
        log_pred_map.at(id)->trajectories().Get(0).av_relation());
    auto new_traj = AlignPredictedTrajectoryPoints(
        raw_traj, log_pred_map.at(id)->perception_object().timestamp(),
        prophnet_predictor_input.predict_start_time, kPredictionDuration,
        kPredictionTimeStep);
    // At least two points.
    QCHECK_GE(new_traj.size(), 2);
    // From t = 0.1 till end
    *gt_traj->mutable_gt_points() = {new_traj.begin() + 1, new_traj.end()};
    *features_proto.mutable_scenarios()->Add() = object_scenarios.at(id);
    *features_proto.mutable_priorities()->Add() = object_priorities.at(id);
  }
  // Put AV traj inside the sample.
  auto *gt_traj = features_proto.mutable_gt_trajs()->Add();
  gt_traj->set_object_id(kAvObjectId);
  std::vector<ApolloTrajectoryPointProto> raw_av_traj;
  for (const auto &pt : av_traj.trajectory_point()) {
    raw_av_traj.push_back(pt);
  }
  const auto raw_av_pred_traj =
      ConvertAVTrajectoryPointsToPredictedTrajectoryPoints(raw_av_traj);
  auto new_traj = AlignPredictedTrajectoryPoints(
      raw_av_pred_traj, av_traj.trajectory_start_timestamp(),
      prophnet_predictor_input.predict_start_time, kPredictionDuration,
      kPredictionTimeStep);
  // At least two points.
  QCHECK_GE(new_traj.size(), 2);
  *gt_traj->mutable_gt_points() = {new_traj.begin() + 1, new_traj.end()};

  *features_proto.mutable_labels() = GenerateLabels(
      prediction_context, prophnet_predictor_input.predict_ids, log_pred_map);
  return features_proto;
}

ProphnetDumpedFeatureProto ToProphnetDumpedFeatureProtoWithoutGroundTruth(
    const ProphnetPredictorInput &prophnet_predictor_input,
    const std::map<ObjectIDType, ObjectPredictionScenario> &object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriority> &object_priorities) {
  ProphnetDumpedFeatureProto features_proto;
  auto *mutable_objects_feature = features_proto.mutable_objects_feature();
  const auto &actors_feature =
      prophnet_predictor_input.input_features.actors_feature;
  *mutable_objects_feature->mutable_ego_pos() = {actors_feature.ego_pos.begin(),
                                                 actors_feature.ego_pos.end()};
  *mutable_objects_feature->mutable_rot_mat() = {actors_feature.rot_mat.begin(),
                                                 actors_feature.rot_mat.end()};
  *mutable_objects_feature->mutable_trajs() = {actors_feature.trajs.begin(),
                                               actors_feature.trajs.end()};
  *mutable_objects_feature->mutable_speeds() = {actors_feature.speeds.begin(),
                                                actors_feature.speeds.end()};
  *mutable_objects_feature->mutable_headings() = {
      actors_feature.headings.begin(), actors_feature.headings.end()};
  *mutable_objects_feature->mutable_types() = {actors_feature.types.begin(),
                                               actors_feature.types.end()};
  *mutable_objects_feature->mutable_cur_poses() = {
      actors_feature.cur_poses.begin(), actors_feature.cur_poses.end()};

  *mutable_objects_feature->mutable_shapes() = {actors_feature.shapes.begin(),
                                                actors_feature.shapes.end()};
  *mutable_objects_feature->mutable_max_curvatures() = {
      actors_feature.max_ks.begin(), actors_feature.max_ks.end()};
  *mutable_objects_feature->mutable_max_lateral_accel() = {
      actors_feature.max_lat_accs.begin(), actors_feature.max_lat_accs.end()};
  *mutable_objects_feature->mutable_dist_to_racs() = {
      actors_feature.dist_to_racs.begin(), actors_feature.dist_to_racs.end()};
  auto *mutable_lanes_feature = features_proto.mutable_lanes_feature();
  const auto &lanes_feature =
      prophnet_predictor_input.input_features.lanes_feature;
  *mutable_lanes_feature->mutable_lane_centers() = {
      lanes_feature.lane_centers.begin(), lanes_feature.lane_centers.end()};
  *mutable_lanes_feature->mutable_lane_segments() = {
      lanes_feature.lane_segments.begin(), lanes_feature.lane_segments.end()};
  *mutable_lanes_feature->mutable_lane_controls() = {
      lanes_feature.lane_controls.begin(), lanes_feature.lane_controls.end()};
  *mutable_lanes_feature->mutable_lane_lights() = {
      lanes_feature.lane_lights.begin(), lanes_feature.lane_lights.end()};

  // last id is av.
  for (int i = 0; i < prophnet_predictor_input.predict_ids.size() - 1; ++i) {
    const auto &id = prophnet_predictor_input.predict_ids[i];
    mutable_objects_feature->add_object_id(id);
    *features_proto.mutable_scenarios()->Add() = object_scenarios.at(id);
    *features_proto.mutable_priorities()->Add() = object_priorities.at(id);
  }
  // Put AV traj inside the sample.
  mutable_objects_feature->add_object_id(kAvObjectId);

  return features_proto;
}
}  // namespace prediction
}  // namespace qcraft
