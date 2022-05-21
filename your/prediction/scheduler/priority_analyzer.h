#ifndef ONBOARD_PREDICTION_SCHEDULER_PRIORITY_ANALYZER_H_
#define ONBOARD_PREDICTION_SCHEDULER_PRIORITY_ANALYZER_H_
#include <map>
#include <string>
#include <vector>

#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
std::map<ObjectIDType, ObjectPredictionPriority> AnalyzePriorities(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& obj_scenarios);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_SCHEDULER_PRIORITY_ANALYZER_H_
