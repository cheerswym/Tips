#include "onboard/perception/tracker/motion_filter_2/car_model.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(CarModel, CarState) {
  CarState s;
  EXPECT_DOUBLE_EQ(s.x(), 0.0);
  EXPECT_DOUBLE_EQ(s.y(), 0.0);
  EXPECT_DOUBLE_EQ(s.vel(), 0.0);
  EXPECT_DOUBLE_EQ(s.acc(), 0.0);
  EXPECT_DOUBLE_EQ(s.yaw(), 0.0);
  EXPECT_DOUBLE_EQ(s.yawd(), 0.0);

  s.x() = 1.0;
  s.y() = 2.0;
  s.vel() = 3.0;
  s.acc() = 4.0;
  s.yaw() = 5.0;
  s.yawd() = 6.0;

  CarState p1 = s;

  EXPECT_DOUBLE_EQ(p1.x(), 1.0);
  EXPECT_DOUBLE_EQ(p1.y(), 2.0);
  EXPECT_DOUBLE_EQ(p1.vel(), 3.0);
  EXPECT_DOUBLE_EQ(p1.acc(), 4.0);
  EXPECT_DOUBLE_EQ(p1.yaw(), 5.0);
  EXPECT_DOUBLE_EQ(p1.yawd(), 6.0);
}

TEST(CarModel, CarModelCP) {
  CarState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  CarModelCP model{};
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vel(), 0.0);
  EXPECT_DOUBLE_EQ(s.acc(), 0.0);
  EXPECT_DOUBLE_EQ(s.yaw(), 1.0);
  EXPECT_DOUBLE_EQ(s.yawd(), 0.0);
}

TEST(CarModel, CarModelCV) {
  CarState s;
  s.x() = 1.0;
  s.y() = 1.0;
  s.vel() = 1.0;
  s.yaw() = 0.0;
  CarModelCV model{};
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.1);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vel(), 1.0);
  EXPECT_DOUBLE_EQ(s.acc(), 0.0);
  EXPECT_DOUBLE_EQ(s.yaw(), 0.0);
  EXPECT_DOUBLE_EQ(s.yawd(), 0.0);
}

TEST(CarModel, CarModelCA) {
  CarState s;
  s.x() = 1.0;
  s.y() = 1.0;
  s.vel() = 1.0;
  s.yaw() = 0.0;
  s.acc() = 1.0;
  CarModelCA model{};
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.105);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vel(), 1.1);
  EXPECT_DOUBLE_EQ(s.acc(), 1.0);
  EXPECT_DOUBLE_EQ(s.yaw(), 0.0);
  EXPECT_DOUBLE_EQ(s.yawd(), 0.0);
}

TEST(CarModel, CarModelCTRV) {
  CarState s;
  s.x() = 1.0;
  s.y() = 1.0;
  s.vel() = 1.0;
  s.yaw() = 0.0;
  s.acc() = 1.0;
  s.yawd() = 0.1;
  CarModelCTRV model{};
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  // EXPECT_DOUBLE_EQ(s.x(), 1.105);
  // EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vel(), 1.0);
  EXPECT_DOUBLE_EQ(s.acc(), 0.0);
  EXPECT_DOUBLE_EQ(s.yaw(), 0.01);
  EXPECT_DOUBLE_EQ(s.yawd(), 0.1);

  auto Q = model.ComputeProcessNoise(s, dt);
  QLOG(INFO) << "Q: \n" << Q;
}

TEST(CarModel, Params) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  CarModelCA model(params);

  const double dt = 0.1;
  CarState s;
  s.x() = 1.0;
  s.y() = 1.0;
  s.vel() = 1.0;

  auto Q = model.ComputeProcessNoise(s, dt);
  QLOG(INFO) << "Q: \n" << Q;
  using S = CarState;
  EXPECT_DOUBLE_EQ(Q(S::X, S::X), Sqr(0.5) * 1e-6 / 36.0);
}

}  // namespace qcraft::tracker
