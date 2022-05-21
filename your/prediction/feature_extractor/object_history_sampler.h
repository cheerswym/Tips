#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_

#include <vector>

#include "onboard/prediction/container/object_history_span.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {
std::vector<ObjectProto> ResampleObjectHistorySpan(
    const ObjectHistorySpan &obj_history, double current_ts, double time_step,
    int max_steps);
ObjectProto LerpObjectProto(const ObjectProto &a, const ObjectProto &b,
                            double alpha);
bool CheckHistoryConsistency(const ObjectProto &obj,
                             const ObjectProto &next_obj, double ts);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_
