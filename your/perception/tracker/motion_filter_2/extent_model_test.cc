#include "onboard/perception/tracker/motion_filter_2/extent_model.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(ExtentModel, BBoxState) {
  BBoxState s;
  EXPECT_DOUBLE_EQ(s.length(), 0.0);
  EXPECT_DOUBLE_EQ(s.width(), 0.0);
  EXPECT_DOUBLE_EQ(s.height(), 0.0);

  s.length() = 1.0;
  s.width() = 2.0;
  s.height() = 3.0;

  const auto p1 = s;

  EXPECT_DOUBLE_EQ(p1.length(), 1.0);
  EXPECT_DOUBLE_EQ(p1.width(), 2.0);
  EXPECT_DOUBLE_EQ(p1.height(), 3.0);
}

TEST(ExtentModel, BBoxModel) {
  BBoxState s;
  s << 1.0, 1.0, 1.0;
  BBoxModel model{};
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.length(), 1.0);
  EXPECT_DOUBLE_EQ(s.width(), 1.0);
  EXPECT_DOUBLE_EQ(s.height(), 1.0);
}

TEST(ExtentModel, BBoxModelParams) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  BBoxModel model(params);

  const double dt = 0.1;
  BBoxState s;
  s.length() = 1.0;
  s.width() = 1.0;
  s.height() = 1.0;

  auto Q = model.ComputeProcessNoise(dt);
  QLOG(INFO) << "Q: \n" << Q;
  using S = BBoxState;
  EXPECT_DOUBLE_EQ(Q(S::LENGTH, S::LENGTH), Sqr(0.5) * dt * dt);
  EXPECT_DOUBLE_EQ(Q(S::WIDTH, S::WIDTH), Sqr(0.5) * dt * dt);
  EXPECT_DOUBLE_EQ(Q(S::HEIGHT, S::HEIGHT), Sqr(0.5) * dt * dt);
}

}  // namespace qcraft::tracker
