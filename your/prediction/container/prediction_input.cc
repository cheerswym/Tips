#include "onboard/prediction/container/prediction_input.h"

#include <sstream>

#include "onboard/global/clock.h"
#include "onboard/utils/proto_util.h"
namespace qcraft::prediction {

absl::Status FillPredictionState(const PredictionInput& prediction_input,
                                 PredictionState* prediction_state) {
  if (prediction_state == nullptr) {
    return absl::FailedPreconditionError("prediction_state cannot be empty.");
  }

  if (prediction_input.pose == nullptr) {
    return absl::FailedPreconditionError(
        "prediction_input.pose cannot be empty.");
  }

  if (prediction_input.virtual_objects != nullptr) {
    prediction_state->PushObjectsInputSeq(
        prediction_input.virtual_objects->header().timestamp(),
        prediction_input.virtual_objects->header().seq_number());
  }
  if (prediction_input.real_objects != nullptr) {
    prediction_state->PushObjectsInputSeq(
        prediction_input.real_objects->header().timestamp(),
        prediction_input.real_objects->header().seq_number());
  }

  prediction_state->PushAvInputSeq(
      prediction_input.pose->header().timestamp(),
      prediction_input.pose->header().seq_number());

  prediction_state->prediction_init_time =
      prediction_input.prediction_init_time;

// seq
#define PREDICTION_SEQ_NUM(msg)                                        \
  do {                                                                 \
    if (prediction_input.msg != nullptr) {                             \
      prediction_state->prediction_seq.mutable_##msg()->set_ts_micros( \
          prediction_input.msg->header().timestamp());                 \
      prediction_state->prediction_seq.mutable_##msg()->set_seq_num(   \
          prediction_input.msg->header().seq_number());                \
    }                                                                  \
  } while (false)

  PREDICTION_SEQ_NUM(pose);
  PREDICTION_SEQ_NUM(localization_transform);
  PREDICTION_SEQ_NUM(real_objects);
  PREDICTION_SEQ_NUM(virtual_objects);
  PREDICTION_SEQ_NUM(traffic_light_states);
#undef PREDICTION_SEQ_NUM

  return absl::OkStatus();
}

}  // namespace qcraft::prediction
