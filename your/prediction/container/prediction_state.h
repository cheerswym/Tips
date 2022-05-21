#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_STATE_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_STATE_H_

#include <optional>
#include <string>
#include <utility>

#include "absl/time/time.h"
#include "boost/circular_buffer.hpp"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/prediction/container/objects_history.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::prediction {

struct PredictionState {
  // The intermidate state as Prediction runner input.
  boost::circular_buffer<planner::TimeSeqNum> av =
      boost::circular_buffer<planner::TimeSeqNum>(50);

  // only use 3 seconds, no more than 3*10*2
  boost::circular_buffer<planner::TimeSeqNum> objects =
      boost::circular_buffer<planner::TimeSeqNum>(80);

  absl::Time prediction_init_time;

  // seq
  planner::PredictionSeq prediction_seq;
  void SetObjectsPredicitonTimeSeq(uint64 ts_micros, uint64 seq_num) noexcept;
  void PushObjectsInputSeq(uint64 ts_micros, uint64 seq_num) noexcept;
  void PushAvInputSeq(uint64 ts_micros, uint64 seq_num) noexcept;
  void FromProto(const planner::PredictionStateProto& proto);
  void ToProto(planner::PredictionStateProto* proto) const;
  bool operator==(const PredictionState& other) const;
  void Reset() {
    av.clear();
    objects.clear();
    prediction_seq.Clear();
  }
};

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_STATE_H_
