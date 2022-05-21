#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_HISTORY_SPAN_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_HISTORY_SPAN_H_
#include <string>

#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/utils/elements_history.h"

namespace qcraft {
namespace prediction {
class ObjectPredictionHistorySpan
    : public elements_history::CircularSpan<
          elements_history::Node<double, ObjectPredictionResult>> {
 public:
  using CircularSpan<
      elements_history::Node<double, ObjectPredictionResult>>::CircularSpan;
  void Update(const ObjectPredictionResult& obj_pred_status);
  const ObjectIDType& id() const { return back().val.id; }
  ObjectType type() const { return back().val.perception_object.type(); }
  double timestamp() const { return back().val.perception_object.timestamp(); }
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_HISTORY_SPAN_H_
