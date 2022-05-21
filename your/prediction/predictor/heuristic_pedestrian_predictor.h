#ifndef ONBOARD_PREDICTION_PREDICTOR_HEURISTIC_PEDESTRIAN_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_HEURISTIC_PEDESTRIAN_PREDICTOR_H_

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/predicted_trajectory.h"

namespace qcraft {
namespace prediction {
PredictedTrajectory MakeHeuristicPedestrianPrediction(
    const ObjectHistory* obj, const SemanticMapManager& semantic_map_mgr,
    ObjectPredictionPriority priority);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_HEURISTIC_PEDESTRIAN_PREDICTOR_H_
