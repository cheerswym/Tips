#include "onboard/perception/tracker/motion_filter_2/utils.h"

#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"

namespace qcraft::tracker {
TEST(Function, is_nan) {
  PointState p;
  EXPECT_FALSE(CheckNaN(p));
  p.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(CheckNaN(p));

  CarState c;
  EXPECT_FALSE(CheckNaN(c));
  c.yaw() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(CheckNaN(c));
}

TEST(Function, CheckDivergence) {
  PointState s1;
  Covariance<PointState> p1;
  p1.setZero();
  EXPECT_FALSE(CheckDivergence(p1));
  p1.setConstant(-0.1);
  EXPECT_TRUE(CheckDivergence(p1));

  CarState s2;
  Covariance<CarState> p2;
  p2.setZero();
  EXPECT_FALSE(CheckDivergence(p2));
  p2.setConstant(-0.1);
  EXPECT_TRUE(CheckDivergence(p2));
}

TEST(ComputeMDistanceTest, ShouldPass) {
  // Check heading measurement
  const double z_diff = d2r(150.0);
  const double var = 0.3;
  HeadingMeasurement yaw_diff(z_diff);
  Covariance<HeadingMeasurement> S(var);
  QLOG(INFO) << "Mahalanobis distance: " << ComputeMDistance(yaw_diff, S);
  EXPECT_DOUBLE_EQ(ComputeMDistance(yaw_diff, S), z_diff * z_diff / var);

  // Check pos measurement
  PositionMeasurement pos_diff(0.5, 0.1);
  Covariance<PositionMeasurement> S_pos;
  S_pos << 1.0, 0.0, 0.0, 3.0;

  QLOG(INFO) << "Mahalanobis distance: " << ComputeMDistance(pos_diff, S_pos);
  EXPECT_DOUBLE_EQ(ComputeMDistance(pos_diff, S_pos), 0.25 + 0.01 / 3.0);
}

TEST(IsInValidationGateByMahalanobisDistance, ShouldPass) {
  EXPECT_TRUE(IsInValidationGateByMahalanobisDistance(3.0, 1));
  EXPECT_FALSE(IsInValidationGateByMahalanobisDistance(5.0, 1));
  EXPECT_TRUE(IsInValidationGateByMahalanobisDistance(3.0, 2));
  EXPECT_FALSE(IsInValidationGateByMahalanobisDistance(6.0, 2));
}
}  // namespace qcraft::tracker
