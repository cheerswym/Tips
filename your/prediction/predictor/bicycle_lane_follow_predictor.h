#ifndef ONBOARD_PREDICTION_PREDICTOR_BICYCLE_LANE_FOLLOW_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_BICYCLE_LANE_FOLLOW_PREDICTOR_H_

#include <vector>

#include "onboard/prediction/container/prediction_context.h"
namespace qcraft {
namespace prediction {
std::vector<PredictedTrajectory> MakeBicycleLaneFollowPrediction(
    const ObjectHistory* obj, const PredictionContext& context,
    ObjectPredictionPriority priority);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_BICYCLE_LANE_FOLLOW_PREDICTOR_H_
