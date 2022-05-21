#include "onboard/prediction/long_term/long_term_behavior_estimator.h"

#include <algorithm>
namespace qcraft {
namespace prediction {
ObjectLongTermBehaviorProto EstimateLongTermBehavior(
    const ObjectHistory& object_history) {
  const auto hist = object_history.GetHistory().value();
  QCHECK_GT(hist.size(), 0);
  ObjectLongTermBehaviorProto res;
  res.set_observation_duration(hist.back().time - hist.front().time);
  const double sum_v = std::accumulate(
      hist.begin(), hist.end(), 0.0,
      [](const double val, const auto& obj) { return val + obj.val.v(); });
  res.set_average_speed(sum_v / hist.size());
  return res;
}
}  // namespace prediction
}  // namespace qcraft
