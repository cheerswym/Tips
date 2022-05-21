
#include "onboard/perception/tracker/motion_filter_2/kalman_filter.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(KalmanFilter, ComputeMahalanobisDistanceFromPredictionInMeasurementSpace) {
  PointState state;
  state << 1.0, 1.0, 1.0, 1.0, 0.0, 0.0;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));

  KalmanFilter<PointModelCV> kf(params);

  kf.Init(state);

  const double dt = 0.1;
  PosSpeedMeasurement pos_meas(1.1, 1.1, 1.2, 1.0);
  const double m_distance =
      kf.ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
          pos_meas, kf.state(), kf.state_covariance(), dt, params);
  QLOG(ERROR) << "m_distance=" << m_distance;
  EXPECT_LT(m_distance, 0.04);
}

TEST(FunctionTest, Predict) {
  PointState state;
  state << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));

  KalmanFilter<PointModelCV> kf(params);

  kf.Init(state);

  auto s = kf.state();
  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.ax(), 1.0);
  EXPECT_DOUBLE_EQ(s.ay(), 1.0);

  const double dt = 0.1;
  kf.Predict(dt);
  kf.Predict(dt);
  auto state_data = kf.ComputePrediction(kf.state(), kf.state_covariance(), dt);
  s = kf.Predict(dt);
  QLOG(INFO) << "S_dt_dt_dt: \n" << kf.state();
  QLOG(INFO) << "P_dt_dt_dt: \n" << kf.state_covariance();
  EXPECT_TRUE(kf.state().isApprox(state_data.state()));
  EXPECT_TRUE(kf.state_covariance().isApprox(state_data.state_cov()));
  EXPECT_DOUBLE_EQ(s.x(), 1.3);
  EXPECT_DOUBLE_EQ(s.y(), 1.3);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.ax(), 0.0);
  EXPECT_DOUBLE_EQ(s.ay(), 0.0);

  PositionMeasurement pos(1.31, 1.32);
  EXPECT_LE(kf.ComputeMahalanobisDistance(pos), 1.0);
  PosSpeedMeasurement pos_speed(1.31, 1.32, 1.1, 0.9);
  kf.Update(pos_speed);
  // kf.Update(pos_meas);
  QLOG(INFO) << "S_update: \n" << kf.state();
  QLOG(INFO) << "P_update: \n" << kf.state_covariance();
  EXPECT_FALSE(kf.state().isApprox(state_data.state()));
  EXPECT_FALSE(kf.state_covariance().isApprox(state_data.state_cov()));
}

TEST(KalmanFilter, Constructor) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));
  KalmanFilter<PointModelCV> kf(params);
  EXPECT_FALSE(kf.DebugIsEnabled());
  KalmanFilter<PointModelCV> kf1(params, false);
  EXPECT_FALSE(kf1.DebugIsEnabled());
  KalmanFilter<PointModelCV> kf2(params, true);
  EXPECT_TRUE(kf2.DebugIsEnabled());
}

}  // namespace qcraft::tracker
