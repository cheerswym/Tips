#include "onboard/prediction/scheduler/scheduler.h"

#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/common/multi_timer_util.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature_extractor.h"
#include "onboard/prediction/long_term/long_term_behavior_estimator.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/bicycle_lane_follow_predictor.h"
#include "onboard/prediction/predictor/heuristic_pedestrian_predictor.h"
#include "onboard/prediction/predictor/kinematic_predictor.h"
#include "onboard/prediction/predictor/predictor_util.h"
#include "onboard/prediction/predictor/prophnet_predictor.h"
#include "onboard/prediction/predictor/tnt_predictor.h"
#include "onboard/prediction/predictor/vehicle_lane_follow_predictor.h"
#include "onboard/prediction/scheduler/priority_analyzer.h"
#include "onboard/prediction/scheduler/scenario_analyzer.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/utils/map_util.h"
DEFINE_bool(extract_prophnet_feature, false, "Extract prophnet feature.");
DEFINE_bool(extract_vehicle_tnt_feature, false, "Extract Vehicle TNT feature.");
DEFINE_bool(print_prediction_time_stats, false, "Print prediction time stats.");

namespace qcraft {
namespace prediction {
namespace {
// Used for feature extraction
constexpr int kMinimalFutureTrajPtNum = 15;     // at least 1.5s future traj.
constexpr double kReductionRadiusForTnt = 1.5;  // m.

using PredictorTrajectories =
    std::map<PredictionType, std::vector<PredictedTrajectory>>;
const std::map<PredictionType, int> kPredictorPriorityMap = {
    {PT_STATIONARY, 0},
    {PT_REVERSE_CYCV, 0},
    {PT_PROPHNET, 1},
    {PT_PED_KINEMATIC, 2},
    {PT_BIKE_LANE_FOLLOW, 2},
    {PT_VEHICLE_LANE_FOLLOW, 2},
    {PT_CYCV, 3},
    {PT_CTRA, 3},
    {PT_VOID, 98},
    {PT_ORACLE, 99},
};
// Check if current predictor has higher priority than input predictor.
//  HIGH PRIORITY ===> LOW PRIORITY   0 ===> 99
bool IsHighOrEqualPriority(PredictionType cur, PredictionType in) {
  const auto* cur_priority = FindOrNull(kPredictorPriorityMap, cur);
  const auto* in_priority = FindOrNull(kPredictorPriorityMap, in);
  QCHECK_NOTNULL(cur_priority);
  QCHECK_NOTNULL(in_priority);
  if (*cur_priority <= *in_priority) {
    return true;
  }
  return false;
}
// Try insert the predicted trajectories if predictor relation is satisfied.
void TryInsert(
    std::map<std::string, std::vector<PredictedTrajectory>>* obj_traj_map,
    std::vector<PredictedTrajectory> trajs, const ObjectIDType obj_id) {
  QCHECK_GE(trajs.size(), 1);
  const auto* prev_trajs = FindOrNull(*obj_traj_map, obj_id);
  if (prev_trajs != nullptr) {
    const auto prev_type = prev_trajs->front().type();
    const auto cur_type = trajs.front().type();
    if (IsHighOrEqualPriority(prev_type, cur_type)) {
      return;
    }
  }
  (*obj_traj_map)[obj_id] = std::move(trajs);
}
}  // namespace.
std::map<ObjectIDType, ObjectPredictionResult> SchedulePrediction(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ConflictResolverParams& conflict_resolver_params,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool, PredictionDebugProto* debug) {
  FUNC_QTRACE();
  ScopedMultiTimer timer("PredictionScheduler::SchedulePrediction");

  const auto object_scenarios =
      AnalyzeScenarios(prediction_context, objs_to_predict);
  const auto object_priorities =
      AnalyzePriorities(prediction_context, objs_to_predict, object_scenarios);
  timer.Mark(
      "PredictionScheduler::SchedulePrediction: analyze scenarios & "
      "priorities");

  std::map<ObjectIDType, std::vector<PredictedTrajectory>> object_trajs;

  // 1. Filter out stationary & reverse driving objects and assign them
  // stationary/reverse prediction.
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto& obj = objs_to_predict[i];
    if (obj->GetHistory()->back().val.IsStationary()) {
      auto trajs = {MakeStationaryPrediction(objs_to_predict[i],
                                             kPredictionDuration,
                                             object_priorities.at(obj->id()))};
      TryInsert(&object_trajs, std::move(trajs), obj->id());
      continue;
    }
    if (obj->GetHistory()->back().val.IsReversed()) {
      auto trajs = {MakeReverseCYCVPrediction(
          objs_to_predict[i], *prediction_context.semantic_map_manager(),
          kPredictionDuration, object_priorities.at(obj->id()))};
      TryInsert(&object_trajs, std::move(trajs), obj->id());
      continue;
    }
  }
  timer.Mark(
      "PredictionScheduler::SchedulePrediction:stationary & reverse "
      "prediction");

  // 2. Do Prophnet prediction.
  if (FLAGS_prediction_enable_ml && FLAGS_prediction_enable_prophnet) {
    ProphnetPredictionInput prophnet_prediction_input =
        PrepareProphnetPredictionInput(
            prediction_context, objs_to_predict, object_scenarios,
            object_priorities,
            FLAGS_prediction_prophnet_predict_number_threshold);

    if (!prophnet_prediction_input.objs.empty()) {
      ProphnetPredictorInput prophnet_predictor_input =
          PrepareProphnetPredictorInput(prophnet_prediction_input);
      if (!prophnet_predictor_input.input_features.lanes_feature.lane_centers
               .empty()) {
        auto pred_trajs = MakeProphnetPrediction(
            prophnet_prediction_input, prophnet_predictor_input,
            model_pool.GetProphNetPredictor());
        for (auto& pair : pred_trajs) {
          TryInsert(&object_trajs, std::move(pair.second), pair.first);
        }
        if (FLAGS_prediction_dumping_model_feature) {
          *debug->mutable_features()->mutable_prophnet_data() =
              ToProphnetDumpedFeatureProtoWithoutGroundTruth(
                  prophnet_predictor_input, object_scenarios,
                  object_priorities);
        }
      }
    }
  }
  timer.Mark("PredictionScheduler::SchedulePrediction: prophnet prediction");
  // 3. Final stage of prediction, use heuristic predictors for all
  // non-predicted objs.
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto& obj = objs_to_predict[i];
    if (object_trajs.count(obj->id()) != 0) {
      continue;
    }
    const auto obj_type = GuessType(*obj);
    // 3.1. Do heuristic pedestrian prediction.
    if (obj_type == OT_PEDESTRIAN) {
      auto trajs = {MakeHeuristicPedestrianPrediction(
          objs_to_predict[i], *prediction_context.semantic_map_manager(),
          object_priorities.at(obj->id()))};
      object_trajs[obj->id()] = std::move(trajs);
      continue;
    }

    // 3.2. Do heuristic bike prediction (for non-predicted bikes).
    if (obj_type == OT_MOTORCYCLIST || obj_type == OT_CYCLIST) {
      if (object_trajs.count(obj->id()) == 0) {
        auto trajs = MakeBicycleLaneFollowPrediction(
            objs_to_predict[i], prediction_context,
            object_priorities.at(obj->id()));
        object_trajs[obj->id()] = std::move(trajs);
      }
      continue;
    }

    // 3.3. Do vehicle lane follow prediction (for non-predicted vehicles).
    if (obj_type == OT_VEHICLE) {
      if (object_trajs.count(obj->id()) == 0) {
        auto trajs = MakeVehicleLaneFollowPrediction(
            objs_to_predict[i], prediction_context,
            object_scenarios.at(obj->id()), object_priorities.at(obj->id()));
        object_trajs[obj->id()] = std::move(trajs);
      }
      continue;
    }

    // 3.4. Default obj: const yaw const velocity prediction.
    auto trajs = {MakeCYCVPrediction(
        objs_to_predict[i], *prediction_context.semantic_map_manager(),
        kPredictionDuration, object_priorities.at(obj->id()))};
    object_trajs[obj->id()] = std::move(trajs);
  }
  timer.Mark("PredictionScheduler::SchedulePrediction: heuristic prediction");

  std::map<ObjectIDType, ObjectPredictionResult> results;
  // todo(xiangjun): To be parallelized if necessary.
  const auto& long_term_mgr =
      prediction_context.object_long_term_history_manager();
  for (int i = 0; i < objs_to_predict.size(); ++i) {
    const auto hist_or = objs_to_predict[i]->GetHistory();
    QCHECK(hist_or.ok());
    const auto& hist = hist_or.value();
    const auto& id = hist.id();
    auto* traj = FindOrNull(object_trajs, id);
    if (traj) {
      ObjectLongTermBehaviorProto long_term_behavior;
      const auto* long_term_hist = long_term_mgr.FindOrNull(id);
      if (long_term_hist != nullptr) {
        long_term_behavior = EstimateLongTermBehavior(*long_term_hist);
      }
      results[id] = ObjectPredictionResult{
          .id = id,
          .priority = object_priorities.at(id),
          .scenario = object_scenarios.at(id),
          .perception_object = hist.back().val.object_proto(),
          .stop_time_info = objs_to_predict[i]->GetStopTimeInfo(),
          .long_term_behavior = long_term_behavior,
          .trajectories = {std::move(object_trajs[id])}};
    }
  }

  timer.Mark("PredictionScheduler::SchedulePrediction: results");
  if (FLAGS_print_prediction_time_stats) {
    planner::PrintMultiTimerReportStat(timer);
  }
  QCHECK_EQ(results.size(), objs_to_predict.size())
      << "Prediction input object size does not match output size.";
  return ResolveConflict(results, prediction_context, conflict_resolver_params,
                         debug->mutable_conflict_resolver_debug(), thread_pool);
}

PredictionFeaturesProto SchedulePredictionFeatureExtraction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const ObjectsPredictionProto& log_prediction,
    const TrajectoryProto& av_traj) {
  std::map<ObjectIDType, const ObjectPredictionProto*> log_pred_map;
  for (const auto& obj : log_prediction.objects()) {
    log_pred_map[obj.id()] = &obj;
  }
  // Only consider objects that have ground truth for more than
  // kMinimalFutureTrajPtNum s.
  std::vector<const ObjectHistory*> filtered_objs;
  filtered_objs.reserve(objects_history.size());
  for (const auto* obj : objects_history) {
    const auto* obj_pred = FindOrNull(log_pred_map, obj->id());
    if (obj_pred != nullptr && (*obj_pred)->trajectories_size() > 0) {
      const auto& traj = (*obj_pred)->trajectories()[0];
      if (traj.points_size() > kMinimalFutureTrajPtNum) {
        filtered_objs.push_back(obj);
      }
    }
  }
  const auto object_scenarios =
      AnalyzeScenarios(prediction_context, filtered_objs);
  const auto object_priorities =
      AnalyzePriorities(prediction_context, filtered_objs, object_scenarios);

  PredictionFeaturesProto features;

  // Schedule prophnet feature extraction.
  if (FLAGS_extract_prophnet_feature) {
    ProphnetPredictionInput prophnet_prediction_input =
        PrepareProphnetPredictionInput(
            prediction_context, filtered_objs, object_scenarios,
            object_priorities,
            FLAGS_prediction_prophnet_predict_number_threshold);
    ProphnetPredictorInput prophnet_predictor_input =
        PrepareProphnetPredictorInput(prophnet_prediction_input);
    if (!prophnet_predictor_input.input_features.lanes_feature.lane_centers
             .empty()) {
      *features.mutable_prophnet_data() = ToProphnetDumpedFeatureProto(
          prediction_context, prophnet_predictor_input, object_scenarios,
          object_priorities, log_pred_map, av_traj);
    }
  }
  // TODO(shenlong): Add vehicle TNT feature extraction stuff.
  if (FLAGS_extract_vehicle_tnt_feature) {
    absl::flat_hash_set<ObjectType> tnt_pred_types{OT_VEHICLE};
    VehicleTNTFeature tnt_feature;
    const auto& semantic_map_manager =
        *prediction_context.semantic_map_manager();
    const auto& av_context = prediction_context.av_context();
    const ObjectHistorySpan av_history =
        av_context.GetAvObjectHistory().GetHistory().value();
    auto predicted_objs = SelectTntPredictedObjects(
        av_history.back().val.bounding_box(), objects_history, tnt_pred_types,
        FLAGS_prediction_tnt_predict_number_threshold, semantic_map_manager);
    const auto obj_exits_map = SelectObjectsWithValidGroundTruth(
        semantic_map_manager, log_pred_map, &predicted_objs);
    std::map<mapping::ElementId, std::vector<Vec2d>> intersection_exits_map;
    for (const auto& key_val : predicted_objs) {
      if (intersection_exits_map.count(key_val.second->id) != 0) {
        continue;
      }
      intersection_exits_map[key_val.second->id] = GetExitsOfIntersection(
          semantic_map_manager, *key_val.second, kReductionRadiusForTnt);
    }
    const double current_ts = GetCurrentTimeStamp(objects_history);
    const auto resampled_objects_history = GetResampledHistory(
        av_history, objects_history, current_ts, kVehicleTNTConfig.history_num);
    auto tnt_features = PrepareTntFeature(
        predicted_objs, resampled_objects_history, semantic_map_manager,
        intersection_exits_map,
        prediction_context.traffic_light_manager().GetOriginalTlStateMap());
    for (const auto& tnt_feature : tnt_features) {
      const auto& object_prediction_proto =
          *log_pred_map.at(tnt_feature.ego_id);
      auto gt_pair_or = ExtractTntGroundTruth(
          tnt_feature.ref_pose, tnt_feature.rot_rad, object_prediction_proto,
          tnt_feature.targets.targets, current_ts, obj_exits_map);
      if (gt_pair_or.has_value()) {
        auto* tnt_data_proto_ptr = features.add_vehicle_tnt_data();
        ToVehicleTNTDumpedFeatureProto(tnt_feature, tnt_data_proto_ptr);
        *tnt_data_proto_ptr->mutable_gt() = std::move(gt_pair_or->first);

        *tnt_data_proto_ptr->mutable_labels() = std::move(gt_pair_or->second);
      }
    }
  }
  return features;
}

}  // namespace prediction
}  // namespace qcraft
