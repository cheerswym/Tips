#ifndef ONBOARD_PLANNER_UTIL_PREDICTION_UTIL_H_
#define ONBOARD_PLANNER_UTIL_PREDICTION_UTIL_H_

#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace planner {
// Aligns an object's prediction to a given time.
absl::Status AlignPredictionTime(double current_time,
                                 ObjectPredictionProto* aligned_object_pred);

// This function merges latest perception objects with predictions. If an object
// has no prediction, it computes a prediction. If an object already has
// prediction, we update the perception object inside the prediction.
absl::flat_hash_map<std::string_view, ObjectPredictionProto>
JoinPerceptionAndPrediction(const ObjectsProto* latest_perceptions,
                            const ObjectsPredictionProto* predictions,
                            ThreadPool* thread_pool);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_UTIL_PREDICTION_UTIL_H_
