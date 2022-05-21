#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_RUNNER_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_RUNNER_H_

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/container/prediction_input.h"

namespace qcraft::prediction {
absl::Status CheckInput(const PredictionInput &input);

absl::Status PreprocessPredictionInput(PredictionInput *input);

absl::StatusOr<std::shared_ptr<ObjectsPredictionProto>> ComputePrediction(
    const PredictionInput &input, PredictionContext *prediction_context,
    ModelPool *model_pool, ThreadPool *thread_pool,
    PredictionDebugProto *debug);

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_RUNNER_H_
