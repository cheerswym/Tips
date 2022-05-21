#include "onboard/prediction/container/object_predictions_history.h"

#include <algorithm>
#include <limits>

#include "onboard/global/clock.h"
#include "onboard/global/timer.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double ttl = 3.0;
}
void ObjectPredictionsHistory::Update(
    const std::vector<ObjectPredictionResult>& object_prediction_results) {
  double end_t = std::numeric_limits<double>::max();
  for (const auto& result : object_prediction_results) {
    (*this)[result.id].Push(result.perception_object.timestamp(), result);
    end_t = std::min(result.perception_object.timestamp(), end_t);
  }
  if (object_prediction_results.size() > 0) {
    PopBegin(end_t - ttl);
  }
}
}  // namespace prediction
}  // namespace qcraft
