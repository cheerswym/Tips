#ifndef ONBOARD_PREDICTION_PREDICTOR_CONFLICT_RESOLVER_H_
#define ONBOARD_PREDICTION_PREDICTOR_CONFLICT_RESOLVER_H_

#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/conflict_resolver/conflict_resolver_params.h"
#include "onboard/prediction/conflict_resolver/object_conflict_manager.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace prediction {
std::map<ObjectIDType, ObjectPredictionResult> ResolveConflict(
    const std::map<ObjectIDType, ObjectPredictionResult>& predictions,
    const PredictionContext& prediction_context,
    const ConflictResolverParams& params,
    ConflictResolverDebugProto* debug_proto, ThreadPool* thread_pool);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_CONFLICT_RESOLVER_H_
