#ifndef ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_H_
#define ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_H_
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "onboard/async/thread_pool.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft {
namespace prediction {
std::map<ObjectIDType, ObjectPredictionResult> SchedulePrediction(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ConflictResolverParams& conflict_resolver_params,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool, PredictionDebugProto* debug);

PredictionFeaturesProto SchedulePredictionFeatureExtraction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectsPredictionProto& log_prediction,
    const TrajectoryProto& av_traj);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_H_
