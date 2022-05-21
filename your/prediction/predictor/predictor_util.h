#ifndef ONBOARD_PREDICTION_PREDICTOR_PREDICTOR_UTIL_H_
#define ONBOARD_PREDICTION_PREDICTOR_PREDICTOR_UTIL_H_

#include <algorithm>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/types/span.h"
#include "onboard/math/vec.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
std::vector<const ObjectHistory *> ScreenPredictObjectsByDistance(
    const Box2d &ego_box,
    absl::Span<const ObjectHistory *const> objs_to_predict,
    int max_objects_num);

std::vector<const ObjectHistory *> ScreenPredictObjectsByType(
    absl::Span<const ObjectHistory *const> objs_to_predict,
    const absl::flat_hash_set<ObjectType> &pred_types);

double GetCurrentTimeStamp(absl::Span<const ObjectHistory *const> objs);

ResampledObjectsHistory GetResampledHistory(
    const ObjectHistorySpan &av_object,
    absl::Span<const ObjectHistory *const> objs, double current_ts,
    int history_num);
}  // namespace prediction
}  // namespace qcraft
#endif  // ONBOARD_PREDICTION_PREDICTOR_PREDICTOR_UTIL_H_
