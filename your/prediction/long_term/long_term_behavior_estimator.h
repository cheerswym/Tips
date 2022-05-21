#ifndef ONBOARD_PREDICTION_LONG_TERM_LONG_TERM_BEHAVIOR_ESTIMATOR_H_
#define ONBOARD_PREDICTION_LONG_TERM_LONG_TERM_BEHAVIOR_ESTIMATOR_H_
#include "onboard/prediction/container/object_history.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
ObjectLongTermBehaviorProto EstimateLongTermBehavior(
    const ObjectHistory& object_history);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_LONG_TERM_LONG_TERM_BEHAVIOR_ESTIMATOR_H_
