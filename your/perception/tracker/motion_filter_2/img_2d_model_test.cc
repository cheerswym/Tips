#include "onboard/perception/tracker/motion_filter_2/img_2d_model.h"

#include <iostream>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {
TEST(Img2DModelCV, Img2DState) {
  Img2DState s;
  EXPECT_DOUBLE_EQ(s.x(), 0.0);
  EXPECT_DOUBLE_EQ(s.y(), 0.0);
  EXPECT_DOUBLE_EQ(s.a(), 0.0);
  EXPECT_DOUBLE_EQ(s.h(), 0.0);
  EXPECT_DOUBLE_EQ(s.vx(), 0.0);
  EXPECT_DOUBLE_EQ(s.vy(), 0.0);
  EXPECT_DOUBLE_EQ(s.va(), 0.0);
  EXPECT_DOUBLE_EQ(s.vh(), 0.0);

  s.x() = 1.0;
  s.y() = 2.0;
  s.a() = 3.0;
  s.h() = 4.0;
  s.vx() = 5.0;
  s.vy() = 6.0;
  s.va() = 7.0;
  s.vh() = 8.0;

  Img2DState p1 = s;

  EXPECT_DOUBLE_EQ(p1.x(), 1.0);
  EXPECT_DOUBLE_EQ(p1.y(), 2.0);
  EXPECT_DOUBLE_EQ(p1.a(), 3.0);
  EXPECT_DOUBLE_EQ(p1.h(), 4.0);
  EXPECT_DOUBLE_EQ(p1.vx(), 5.0);
  EXPECT_DOUBLE_EQ(p1.vy(), 6.0);
  EXPECT_DOUBLE_EQ(p1.va(), 7.0);
  EXPECT_DOUBLE_EQ(p1.vh(), 8.0);
}

TEST(Img2DModelCV, Img2DModelCV) {
  Img2DState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  Img2DModelCV model;
  const double dt = 0.1;
  s = model.ComputeStateTransition(s, dt);

  EXPECT_DOUBLE_EQ(s.x(), 1.1);
  EXPECT_DOUBLE_EQ(s.y(), 1.1);
  EXPECT_DOUBLE_EQ(s.a(), 1.1);
  EXPECT_DOUBLE_EQ(s.h(), 1.1);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.va(), 1.0);
  EXPECT_DOUBLE_EQ(s.vh(), 1.0);
}

TEST(Img2DModelCV, Params) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/img_2d_model.pb.txt",
      &params));
  Img2DModelCV model(params);

  const double dt = 0.1;
  Img2DState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  auto Q = model.ComputeProcessNoise(s, dt);
  QLOG(INFO) << "Q: \n" << Q;
  using S = Img2DState;
  EXPECT_DOUBLE_EQ(Q(S::X, S::X), Sqr(0.05) * 1.0);
  EXPECT_DOUBLE_EQ(Q(S::Y, S::Y), Sqr(0.05) * 1.0);
  EXPECT_DOUBLE_EQ(Q(S::A, S::A), 1e-2);
  EXPECT_DOUBLE_EQ(Q(S::H, S::H), Sqr(0.05) * 1.0);
  EXPECT_DOUBLE_EQ(Q(S::VEL_X, S::VEL_X), Sqr(0.00625) * 1.0);
  EXPECT_DOUBLE_EQ(Q(S::VEL_Y, S::VEL_Y), Sqr(0.00625) * 1.0);
  EXPECT_DOUBLE_EQ(Q(S::VEL_A, S::VEL_A), 1e-2);
  EXPECT_DOUBLE_EQ(Q(S::VEL_H, S::VEL_H), Sqr(0.00625) * 1.0);
}

}  // namespace qcraft::tracker
