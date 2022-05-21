#ifndef ONBOARD_PREDICTION_PREDICTOR_VEHICLE_LANE_FOLLOW_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_VEHICLE_LANE_FOLLOW_PREDICTOR_H_

#include <vector>

#include "onboard/prediction/container/prediction_context.h"
namespace qcraft {
namespace prediction {
std::vector<PredictedTrajectory> MakeVehicleLaneFollowPrediction(
    const ObjectHistory* obj, const PredictionContext& context,
    const ObjectPredictionScenario& scenario,
    ObjectPredictionPriority priority);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_VEHICLE_LANE_FOLLOW_PREDICTOR_H_
