#ifndef ONBOARD_PREDICTION_PREDICTOR_PROPHNET_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_PROPHNET_PREDICTOR_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/nets/prophnet_predictor.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/prophnet_feature_extractor.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory.pb.h"
namespace qcraft {
namespace prediction {

using ObjectsPredTrajsMap =
    std::map<std::string, std::vector<PredictedTrajectory>>;

struct ProphnetPredictorInput {
  prophnet::ProphnetFeature input_features;
  std::vector<ObjectIDType> predict_ids;
  double predict_start_time;
};

struct ProphnetPredictionInput {
  const ObjectHistorySpan av_obj;
  std::vector<const ObjectHistory*> objs;
  std::vector<ObjectPredictionPriority> priors;
  std::vector<ObjectPredictionScenario> scenarios;
  const SemanticMapManager* semantic_map_mgr;
  const TrafficLightManager::TLStateHashMap* inferred_tl_state_map;
  const TrafficLightManager::TLStateHashMap* original_tl_state_map;
};

ObjectsPredTrajsMap MakeProphnetPrediction(
    const ProphnetPredictionInput& prophnet_prediction_input,
    const ProphnetPredictorInput& prophnet_predictor_input,
    const ProphNetPredictor& prophnet_predictor);

ProphnetPredictionInput PrepareProphnetPredictionInput(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriority>& object_priorities,
    int max_objects_num);

ProphnetPredictorInput PrepareProphnetPredictorInput(
    const ProphnetPredictionInput& prophnet_prediction_input);

ProphnetDumpedFeatureProto ToProphnetDumpedFeatureProto(
    const PredictionContext& prediction_context,
    const ProphnetPredictorInput& prophnet_predictor_input,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriority>& object_priorities,
    const std::map<ObjectIDType, const ObjectPredictionProto*>& log_pred_map,
    const TrajectoryProto& av_traj);

ProphnetDumpedFeatureProto ToProphnetDumpedFeatureProtoWithoutGroundTruth(
    const ProphnetPredictorInput& prophnet_predictor_input,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriority>& object_priorities);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_PROPHNET_PREDICTOR_H_
