
#include "onboard/perception/tracker/motion_filter_2/img_2d_kalman_filter.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/img_2d_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(ComputeMahalanobisDistanceFromPredictionInMeasurementSpace, TestImg2DKF) {
  Img2DState state;
  state << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/img_2d_model.pb.txt",
      &params));

  // TODO(Yan): delete init_time_stamp here and deal this in estimator
  const double init_time = 0.0;

  Img2DKalmanFilter<Img2DModelCV> kf(params, state, init_time);

  const double dt = 0.1;
  Img2DMeasurement img2d_meas(1.1, 1.1, 1.2, 1.1);
  const double m_distance =
      kf.ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
          img2d_meas, kf.state(), kf.state_covariance(), dt, params);
  QLOG(ERROR) << "m_distance=" << m_distance;
  EXPECT_LT(m_distance, 1.0);
}

TEST(FunctionTest, Predict) {
  Img2DState state;
  state << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/img_2d_model.pb.txt",
      &params));

  // TODO(Yan): delete init_time_stamp here and deal this in estimator
  const double init_time = 0.0;
  Img2DKalmanFilter<Img2DModelCV> kf(params, state, init_time);

  auto s = kf.state();
  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.a(), 1.0);
  EXPECT_DOUBLE_EQ(s.h(), 1.0);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.va(), 1.0);
  EXPECT_DOUBLE_EQ(s.vh(), 1.0);

  const double dt = 0.1;
  kf.Predict(dt);
  kf.Predict(2 * dt);
  auto state_data =
      kf.ComputePrediction(kf.state(), kf.state_covariance(), 3 * dt);
  s = kf.Predict(3 * dt);
  QLOG(INFO) << "S_dt_dt_dt: \n" << kf.state();
  QLOG(INFO) << "P_dt_dt_dt: \n" << kf.state_covariance();
  EXPECT_TRUE(kf.state().isApprox(state_data.state()));
  EXPECT_TRUE(kf.state_covariance().isApprox(state_data.state_cov()));
  EXPECT_DOUBLE_EQ(s.x(), 1.3);
  EXPECT_DOUBLE_EQ(s.y(), 1.3);
  EXPECT_DOUBLE_EQ(s.a(), 1.3);
  EXPECT_DOUBLE_EQ(s.h(), 1.3);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.va(), 1.0);
  EXPECT_DOUBLE_EQ(s.vh(), 1.0);

  Img2DMeasurement img2d_meas(1.31, 1.31, 1.3, 1.3);
  EXPECT_LE(kf.ComputeMahalanobisDistance(img2d_meas), 1.0);
  kf.Update(img2d_meas);
  QLOG(INFO) << "S_update: \n" << kf.state();
  QLOG(INFO) << "P_update: \n" << kf.state_covariance();
  EXPECT_FALSE(kf.state().isApprox(state_data.state()));
  EXPECT_FALSE(kf.state_covariance().isApprox(state_data.state_cov()));
}

}  // namespace qcraft::tracker
