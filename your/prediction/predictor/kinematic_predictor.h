#ifndef ONBOARD_PREDICTION_PREDICTOR_KINEMATIC_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_KINEMATIC_PREDICTOR_H_

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/predicted_trajectory.h"
namespace qcraft {
namespace prediction {
PredictedTrajectory MakeStationaryPrediction(const ObjectHistory* obj,
                                             double prediction_horizon,
                                             ObjectPredictionPriority priority);

PredictedTrajectory MakeCTRAPrediction(const ObjectHistory* obj,
                                       double stop_time,
                                       double prediction_horizon,
                                       ObjectPredictionPriority priority,
                                       bool use_acc);

PredictedTrajectory MakeCYCVPrediction(
    const ObjectHistory* obj, const SemanticMapManager& semantic_map_mgr,
    double prediction_horizon, ObjectPredictionPriority priority);
PredictedTrajectory MakeReverseCYCVPrediction(
    const ObjectHistory*, const SemanticMapManager& semantic_map_mgr,
    double prediction_horizon, ObjectPredictionPriority priority);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_KINEMATIC_PREDICTOR_H_
