#include "onboard/prediction/container/prediction_state.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/utils/proto_util.h"

namespace qcraft::prediction {
namespace {

[[maybe_unused]] std::ostream &operator<<(std::ostream &out,
                                          const PredictionState &state) {
  planner::PredictionStateProto proto;
  state.ToProto(&proto);
  out << proto.ShortDebugString();
  return out;
}

TEST(PredictionStateTest, EqualsTest) {
  PredictionState state;
  state.prediction_init_time = absl::Now();
  TrajectoryProto trajectory;
  trajectory.set_planner_state_seq_num(78);
  planner::PredictionStateProto proto;

  int seq_size = 3;
  for (int i = 0; i < seq_size; i++) {
    planner::TimeSeqNum time_seq;
    time_seq.set_ts_micros(2 * i);
    time_seq.set_seq_num(2 * i + 1);
    state.av.push_back(time_seq);

    time_seq.set_ts_micros(2 * i + 1);
    time_seq.set_seq_num(2 * i);
    state.objects.push_back(time_seq);
  }

  state.prediction_seq.mutable_localization_transform()->set_seq_num(1234);
  state.prediction_seq.mutable_pose()->set_seq_num(2345);
  state.ToProto(&proto);
  PredictionState recover;
  recover.FromProto(proto);
  ASSERT_EQ(proto.av_size(), recover.av.size());
  planner::PredictionStateProto other;
  recover.ToProto(&other);
  ASSERT_TRUE(ProtoEquals(proto, other));
  ASSERT_EQ(state, recover);
}

TEST(PredictionStateTest, PushAvInputSeq) {
  PredictionState state;
  int seq_size = 3;
  for (int i = 0; i < seq_size; i++) {
    state.PushAvInputSeq(2 * i, 2 * i + 1);
  }
  EXPECT_EQ(seq_size, state.av.size());
}
}  // namespace
}  // namespace qcraft::prediction
