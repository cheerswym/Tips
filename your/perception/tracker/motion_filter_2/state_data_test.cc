#include "onboard/perception/tracker/motion_filter_2/state_data.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(Img2DStateData, FunctionTest) {
  // Constructor
  Img2DStateData p;
  EXPECT_DOUBLE_EQ(p.state().x(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().y(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().a(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().h(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().vx(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().vy(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().va(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().vh(), 0.0);

  Img2DState state;
  state << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0;
  Img2DStateData::CovarianceType cov =
      Img2DStateData::CovarianceType::Constant(3.0);
  Img2DStateData p1{state, cov};
  EXPECT_DOUBLE_EQ(p1.state().x(), 1.0);
  EXPECT_DOUBLE_EQ(p1.state().y(), 2.0);
  EXPECT_DOUBLE_EQ(p1.state().a(), 3.0);
  EXPECT_DOUBLE_EQ(p1.state().h(), 4.0);
  EXPECT_DOUBLE_EQ(p1.state().vx(), 5.0);
  EXPECT_DOUBLE_EQ(p1.state().vy(), 6.0);
  EXPECT_DOUBLE_EQ(p1.state().va(), 7.0);
  EXPECT_DOUBLE_EQ(p1.state().vh(), 8.0);

  auto cov_4d = p1.GetStatePosCov();
  EXPECT_DOUBLE_EQ(cov_4d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(0, 3), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(1, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(1, 2), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(1, 3), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(2, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(2, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(2, 2), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(2, 3), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(3, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(3, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(3, 2), 3.0);
  EXPECT_DOUBLE_EQ(cov_4d(3, 3), 3.0);
}

TEST(PointStateData, FunctionTest) {
  // Constructor
  PointStateData p;
  EXPECT_DOUBLE_EQ(p.state().x(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().y(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().vx(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().vy(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().ax(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().ay(), 0.0);

  PointState state;
  state << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  PointStateData::CovarianceType cov =
      PointStateData::CovarianceType::Constant(3.0);
  PointStateData p1{state, cov};
  EXPECT_DOUBLE_EQ(p1.state().x(), 1.0);
  EXPECT_DOUBLE_EQ(p1.state().y(), 2.0);
  EXPECT_DOUBLE_EQ(p1.state().vx(), 3.0);
  EXPECT_DOUBLE_EQ(p1.state().vy(), 4.0);
  EXPECT_DOUBLE_EQ(p1.state().ax(), 5.0);
  EXPECT_DOUBLE_EQ(p1.state().ay(), 6.0);

  auto cov_2d = p1.GetPosCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);

  cov_2d = p1.GetSpeedCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);

  cov_2d = p1.GetAxAyCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);

  auto s = StateData(p1);
  QLOG(INFO) << s;
  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 2.0);
  EXPECT_DOUBLE_EQ(s.GetVx(), 3.0);
  EXPECT_DOUBLE_EQ(s.GetVy(), 4.0);
  EXPECT_DOUBLE_EQ(s.GetAx(), 5.0);
  EXPECT_DOUBLE_EQ(s.GetAy(), 6.0);

  cov_2d = s.GetStatePosCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);

  cov_2d = s.GetSpeedCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);

  cov_2d = s.GetAxAyCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);
}

TEST(CarStateData, FunctionTest) {
  // Constructor
  CarStateData p;
  EXPECT_DOUBLE_EQ(p.state().x(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().y(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().vel(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().yaw(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().yawd(), 0.0);
  EXPECT_DOUBLE_EQ(p.state().acc(), 0.0);

  CarState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.yaw() = 0.0;
  state.vel() = 4.0;
  state.yawd() = 5.0;
  state.acc() = 6.0;
  CarStateData::CovarianceType cov =
      CarStateData::CovarianceType::Constant(3.0);
  CarStateData p1{state, cov};
  EXPECT_DOUBLE_EQ(p1.state().x(), 1.0);
  EXPECT_DOUBLE_EQ(p1.state().y(), 2.0);
  EXPECT_DOUBLE_EQ(p1.state().yaw(), 0.0);
  EXPECT_DOUBLE_EQ(p1.state().vel(), 4.0);
  EXPECT_DOUBLE_EQ(p1.state().yawd(), 5.0);
  EXPECT_DOUBLE_EQ(p1.state().acc(), 6.0);

  auto cov_2d = p1.GetPosCov();
  EXPECT_DOUBLE_EQ(cov_2d(0, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(0, 1), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 0), 3.0);
  EXPECT_DOUBLE_EQ(cov_2d(1, 1), 3.0);

  EXPECT_DOUBLE_EQ(p1.GetVelCov(), 3.0);
  EXPECT_DOUBLE_EQ(p1.GetAccCov(), 3.0);
  EXPECT_DOUBLE_EQ(p1.GetYawCov(), 3.0);
  EXPECT_DOUBLE_EQ(p1.GetYawDCov(), 3.0);

  auto s = StateData(p1);
  QLOG(INFO) << s;
  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 2.0);
  EXPECT_DOUBLE_EQ(s.GetYaw(), 0.0);
  EXPECT_DOUBLE_EQ(s.GetVel(), 4.0);

  cov_2d = s.GetStatePosCov();
  EXPECT_DOUBLE_EQ(s.GetVelCov(), 3.0);
  EXPECT_DOUBLE_EQ(s.GetAccCov(), 3.0);
  EXPECT_DOUBLE_EQ(s.GetYawCov(), 3.0);
  EXPECT_DOUBLE_EQ(s.GetYawDCov(), 3.0);
}

}  // namespace qcraft::tracker
