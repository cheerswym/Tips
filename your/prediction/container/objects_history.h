#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECTS_HISTORY_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECTS_HISTORY_H_
#include <string>

#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/utils/elements_history.h"
namespace qcraft {
namespace prediction {
class ObjectsHistory
    : public elements_history::ElementsHistory<
          std::string, double, PredictionObject, ObjectHistory> {
 public:
  using ElementsHistory<std::string, double, PredictionObject,
                        ObjectHistory>::ElementsHistory;
  explicit ObjectsHistory(size_t history_capacity, double max_hist_time_len,
                          double min_dt)
      : ElementsHistory(history_capacity),
        max_hist_time_len_(max_hist_time_len),
        min_dt_(min_dt) {}
  void Update(const ObjectsProto& objects_proto,
              const LocalizationTransformProto& loc_transform,
              ThreadPool* thread_pool);

 private:
  double max_hist_time_len_;
  double min_dt_;
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECTS_HISTORY_H_
