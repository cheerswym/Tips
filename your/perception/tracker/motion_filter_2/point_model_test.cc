#include "onboard/perception/tracker/motion_filter_2/point_model.h"

#include <iostream>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(PointModel, PointState) {
  PointState ps;
  EXPECT_DOUBLE_EQ(ps.x(), 0.0);
  EXPECT_DOUBLE_EQ(ps.y(), 0.0);
  EXPECT_DOUBLE_EQ(ps.vx(), 0.0);
  EXPECT_DOUBLE_EQ(ps.vy(), 0.0);
  EXPECT_DOUBLE_EQ(ps.ax(), 0.0);
  EXPECT_DOUBLE_EQ(ps.ay(), 0.0);

  ps.x() = 1.0;
  ps.y() = 2.0;
  ps.vx() = 3.0;
  ps.vy() = 4.0;
  ps.ax() = 5.0;
  ps.ay() = 6.0;

  PointState p1 = ps;

  EXPECT_DOUBLE_EQ(p1.x(), 1.0);
  EXPECT_DOUBLE_EQ(p1.y(), 2.0);
  EXPECT_DOUBLE_EQ(p1.vx(), 3.0);
  EXPECT_DOUBLE_EQ(p1.vy(), 4.0);
  EXPECT_DOUBLE_EQ(p1.ax(), 5.0);
  EXPECT_DOUBLE_EQ(p1.ay(), 6.0);
}

TEST(PointModel, PointModelCP) {
  PointState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  PointModelCP model;
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vx(), 0.0);
  EXPECT_DOUBLE_EQ(s.vy(), 0.0);
  EXPECT_DOUBLE_EQ(s.ax(), 0.0);
  EXPECT_DOUBLE_EQ(s.ay(), 0.0);
}

TEST(PointModel, PointModelCV) {
  PointState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  PointModelCV model;
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.1);
  EXPECT_DOUBLE_EQ(s.y(), 1.1);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.ax(), 0.0);
  EXPECT_DOUBLE_EQ(s.ay(), 0.0);
}

TEST(PointModel, PointModelCA) {
  PointState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  PointModelCA model;
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.105);
  EXPECT_DOUBLE_EQ(s.y(), 1.105);
  EXPECT_DOUBLE_EQ(s.vx(), 1.1);
  EXPECT_DOUBLE_EQ(s.vy(), 1.1);
  EXPECT_DOUBLE_EQ(s.ax(), 1.0);
  EXPECT_DOUBLE_EQ(s.ay(), 1.0);
}

TEST(PointModel, Params) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));
  PointModelCA model(params);

  const double dt = 0.1;

  auto Q = model.ComputeProcessNoise(dt);
  using S = PointState;
  EXPECT_DOUBLE_EQ(Q(S::X, S::X), Sqr(0.2) * 1e-6 / 36.0);
}

}  // namespace qcraft::tracker
