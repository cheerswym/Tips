
#include "onboard/perception/tracker/motion_filter_2/ukf.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/car_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(ComputeMahalanobisDistanceFromPredictionInMeasurementSpace, TestUKF) {
  CarState state;
  state.x() = 0.0;
  state.y() = 0.0;
  state.vel() = 1.0;
  state.yaw() = 0.0;
  state.yawd() = 0.0;
  state.acc() = 0.0;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  UKF<CarModelCTRV> ukf(params);

  ukf.Init(state);

  const double dt = 0.1;
  PositionMeasurement pos_meas(0.1, 0.0);
  const double m_distance =
      ukf.ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
          pos_meas, ukf.state(), ukf.state_covariance(), dt, params);
  EXPECT_LT(m_distance, 1e-3);
}

TEST(FunctionTest, All) {
  CarState state;
  state.x() = 0.0;
  state.y() = 0.0;
  state.vel() = 1.0;
  state.yaw() = 0.0;
  state.yawd() = 0.1;
  state.acc() = 0.2;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  UKF<CarModelCTRV> ukf(params);

  ukf.Init(state);

  const double dt = 0.1;
  auto state_data =
      ukf.ComputePrediction(ukf.state(), ukf.state_covariance(), dt);
  QLOG(INFO) << "x_compute_pred: \n" << state_data.state();
  QLOG(INFO) << "P_compute_pred: \n" << state_data.state_cov();
  ukf.Predict(dt);
  QLOG(INFO) << "x_predict: \n" << ukf.state();
  QLOG(INFO) << "P_predict: \n" << ukf.state_covariance();
  EXPECT_TRUE(ukf.state().isApprox(state_data.state()));
  EXPECT_TRUE(ukf.state_covariance().isApprox(state_data.state_cov()));

  PositionMeasurement pos_meas(0.11, 0.05);
  EXPECT_LE(ukf.ComputeMahalanobisDistance(pos_meas), 1.0);
  ukf.Update(pos_meas);

  QLOG(INFO) << "x_update: \n" << ukf.state();
  QLOG(INFO) << "P_update: \n" << ukf.state_covariance();
  EXPECT_FALSE(ukf.state().isApprox(state_data.state()));
  EXPECT_FALSE(ukf.state_covariance().isApprox(state_data.state_cov()));
}

TEST(Update, CombinedAndSeparate) {
  CarState state;
  state.x() = 0.0;
  state.y() = 0.0;
  state.vel() = 1.0;
  state.yaw() = 0.0;
  state.yawd() = 0.1;
  state.acc() = 0.2;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  UKF<CarModelCTRV> ukf(params);

  ukf.Init(state);

  const double dt = 0.1;
  ukf.Predict(dt);
  auto ukf_combined_update = ukf;
  const double x = 0.11;
  const double y = 0.05;
  const double vel = 4.0;

  PositionMeasurement pos_meas(x, y);
  ukf.Update(pos_meas);
  VelocityMeasurement v(vel);
  ukf.Update(v);
  PosVelocityMeasurement pos_vel(x, y, vel);
  ukf_combined_update.Update(pos_vel);

  QLOG(INFO) << "x: \n" << ukf.state();
  QLOG(INFO) << "P: \n" << ukf.state_covariance();
  QLOG(INFO) << "x_combined: \n" << ukf_combined_update.state();
  QLOG(INFO) << "P_combined: \n" << ukf_combined_update.state_covariance();
  EXPECT_FALSE(ukf.state().isApprox(ukf_combined_update.state()));
  EXPECT_FALSE(
      ukf.state_covariance().isApprox(ukf_combined_update.state_covariance()));
}

TEST(UKF, Constructor) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  UKF<CarModelCTRV> ukf(params);
  EXPECT_FALSE(ukf.DebugIsEnabled());
  UKF<CarModelCTRV> ukf1(params, false);
  EXPECT_FALSE(ukf1.DebugIsEnabled());
  UKF<CarModelCTRV> ukf2(params, true);
  EXPECT_TRUE(ukf2.DebugIsEnabled());
}

}  // namespace qcraft::tracker
