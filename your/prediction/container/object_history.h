#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_H_
#include <string>

#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/utils/elements_history.h"
struct StopTimeInfo {
  // move : 0  stop:  how long time the object has stoped.
  double time_duration_since_stop() const { return last_time - stopped_since; }

  // the most recent moving duration
  double last_move_time_duration() const {
    return stopped_since - last_moved_since;
  }

  // the most recent stop duration before the most rencent moving duration.
  double previous_stop_time_duration() const {
    return last_moved_since - prev_stopped_since;
  }
  double last_time = 0.0;
  double stopped_since = 0.0;
  double prev_stopped_since = 0.0;
  double last_moved_since = 0.0;
};

namespace qcraft {
namespace prediction {
class ObjectHistory
    : public elements_history::ElementHistory<double, PredictionObject,
                                              ObjectHistorySpan> {
 public:
  using ElementHistory<double, PredictionObject,
                       ObjectHistorySpan>::ElementHistory;
  const ObjectIDType& id() const {
    const auto hist_or = GetHistory();
    QCHECK_OK(hist_or.status());
    return hist_or->id();
  }
  ObjectType type() const {
    const auto hist_or = GetHistory();
    QCHECK_OK(hist_or.status());
    return hist_or->type();
  }
  double timestamp() const {
    const auto hist_or = GetHistory();
    QCHECK_OK(hist_or.status());
    return hist_or->timestamp();
  }
  bool UpdateStopTimeInfo();
  const StopTimeInfo& GetStopTimeInfo() const { return stop_time_info_; }

 private:
  StopTimeInfo stop_time_info_;
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_HISTORY_H_
