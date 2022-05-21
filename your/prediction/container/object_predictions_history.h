#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTIONS_HISTORY_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTIONS_HISTORY_H_
#include <string>
#include <vector>

#include "onboard/prediction/container/object_prediction_history_span.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
class ObjectPredictionsHistory
    : public elements_history::ElementsHistory<
          std::string, double, ObjectPredictionResult,
          elements_history::ElementHistory<double, ObjectPredictionResult,
                                           ObjectPredictionHistorySpan>> {
 public:
  using ElementsHistory<std::string, double, ObjectPredictionResult,
                        elements_history::ElementHistory<
                            double, ObjectPredictionResult,
                            ObjectPredictionHistorySpan>>::ElementsHistory;
  void Update(
      const std::vector<ObjectPredictionResult>& object_prediction_result);
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTIONS_HISTORY_H_
