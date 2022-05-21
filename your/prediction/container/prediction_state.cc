#include "onboard/prediction/container/prediction_state.h"

#include <sstream>

#include "onboard/global/clock.h"
#include "onboard/utils/proto_util.h"
namespace qcraft::prediction {

namespace {
template <typename C>
bool ContainerEquals(const C& lhs, const C& rhs) {
  auto it = lhs.begin(), it2 = rhs.begin();
  for (; it != lhs.end() && it2 != rhs.end(); ++it, ++it2) {
    if (!(*it == *it2)) {
      return false;
    }
  }
  return it == lhs.end() && it2 == rhs.end();
}
}  // namespace

void PredictionState::SetObjectsPredicitonTimeSeq(uint64 ts_micros,
                                                  uint64 seq_num) noexcept {
  prediction_seq.mutable_objects_prediction()->set_ts_micros(ts_micros);
  prediction_seq.mutable_objects_prediction()->set_seq_num(seq_num);
}
void PredictionState::PushObjectsInputSeq(uint64 ts_micros,
                                          uint64 seq_num) noexcept {
  planner::TimeSeqNum time_seq;
  time_seq.set_ts_micros(ts_micros);
  time_seq.set_seq_num(seq_num);
  objects.push_back(std::move(time_seq));
}

void PredictionState::PushAvInputSeq(uint64 ts_micros,
                                     uint64 seq_num) noexcept {
  planner::TimeSeqNum time_seq;
  time_seq.set_ts_micros(ts_micros);
  time_seq.set_seq_num(seq_num);
  av.push_back(std::move(time_seq));
}

void PredictionState::FromProto(const planner::PredictionStateProto& proto) {
  if (proto.objects_size() > 0) {
    objects.assign(proto.objects().begin(), proto.objects().end());
  }
  if (proto.av_size() > 0) {
    av.assign(proto.av().begin(), proto.av().end());
  }
  prediction_init_time = qcraft::FromProto(proto.prediction_init_time());
  prediction_seq = proto.prediction_seq();
}

void PredictionState::ToProto(planner::PredictionStateProto* proto) const {
  proto->Clear();
  proto->mutable_objects()->Reserve(objects.size());
  for (auto& object : objects) {
    *proto->add_objects() = object;
  }
  proto->mutable_av()->Reserve(av.size());
  for (auto& pose_seq : av) {
    *proto->add_av() = pose_seq;
  }
  qcraft::ToProto(prediction_init_time, proto->mutable_prediction_init_time());
  *proto->mutable_prediction_seq() = prediction_seq;
}

bool PredictionState::operator==(const PredictionState& other) const {
  // av
  if (!ContainerEquals(av, other.av)) {
    return false;
  }

  // objects
  if (!ContainerEquals(objects, other.objects)) {
    return false;
  }

  if (prediction_init_time != other.prediction_init_time) {
    return false;
  }

  if (!ProtoEquals(prediction_seq, other.prediction_seq)) {
    return false;
  }

  return true;
}

}  // namespace qcraft::prediction
