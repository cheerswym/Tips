#include "onboard/perception/tracker/motion_filter_2/motion_filter.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(PointEstimator, Function) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));
  params.set_type(MotionFilterParamProto::POINT_CV);
  std::vector<MotionFilterParamProto> v{params, params};
  IMMProto proto = ToImmProto(v, {0.5, 0.5});
  MotionFilter<PointState> motion_filter(proto);

  PointState state;
  state << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  motion_filter.Init(state);

  QLOG(INFO) << "State: \n" << motion_filter.state();

  auto s = motion_filter.state();

  EXPECT_DOUBLE_EQ(s.x(), 1.0);
  EXPECT_DOUBLE_EQ(s.y(), 1.0);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.ax(), 1.0);
  EXPECT_DOUBLE_EQ(s.ay(), 1.0);

  const double dt = 0.1;
  motion_filter.Predict(dt);
  motion_filter.Predict(dt);
  auto state_data = motion_filter.ComputePrediction(dt);
  motion_filter.Predict(dt);
  EXPECT_TRUE(motion_filter.state().isApprox(state_data.point().state()));
  EXPECT_TRUE(motion_filter.state_covariance().isApprox(
      state_data.point().state_cov()));

  QLOG(INFO) << "S_dt_dt_dt: \n" << motion_filter.state();
  QLOG(INFO) << "P_dt_dt_dt: \n" << motion_filter.state_covariance();

  s = motion_filter.state();
  EXPECT_DOUBLE_EQ(s.x(), 1.3);
  EXPECT_DOUBLE_EQ(s.y(), 1.3);
  EXPECT_DOUBLE_EQ(s.vx(), 1.0);
  EXPECT_DOUBLE_EQ(s.vy(), 1.0);
  EXPECT_DOUBLE_EQ(s.ax(), 0.0);
  EXPECT_DOUBLE_EQ(s.ay(), 0.0);

  PosSpeedMeasurement pos_speed(1.31, 1.32, 1.1, 0.9);
  s = motion_filter.Update(pos_speed);
  QLOG(INFO) << "S_update: \n" << motion_filter.state();
  QLOG(INFO) << "P_update: \n" << motion_filter.state_covariance();
  EXPECT_FALSE(motion_filter.state().isApprox(state_data.point().state()));
  EXPECT_FALSE(motion_filter.state_covariance().isApprox(
      state_data.point().state_cov()));
}

TEST(CarEstimator, Function) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  params.set_type(MotionFilterParamProto::CAR_CV);
  std::vector<MotionFilterParamProto> v{params, params};
  IMMProto proto = ToImmProto(v, {0.5, 0.5});
  MotionFilter<CarState> motion_filter(proto);

  CarState state;
  state.x() = 0.0;
  state.y() = 0.0;
  state.vel() = 1.0;
  state.yaw() = 0.0;
  state.yawd() = 0.1;
  state.acc() = 0.2;

  motion_filter.Init(state);

  QLOG(INFO) << "State: \n" << motion_filter.state();

  auto s = motion_filter.state();

  EXPECT_DOUBLE_EQ(s.x(), 0.0);
  EXPECT_DOUBLE_EQ(s.y(), 0.0);
  EXPECT_DOUBLE_EQ(s.vel(), 1.0);
  EXPECT_DOUBLE_EQ(s.yaw(), 0.0);
  EXPECT_DOUBLE_EQ(s.yawd(), 0.1);
  EXPECT_DOUBLE_EQ(s.acc(), 0.2);

  PositionMeasurement hypo_pos_meas(0.1, 0.1);
  const double dt = 0.1;
  auto state_data = motion_filter.ComputePrediction(dt);
  const auto m_distance =
      motion_filter.ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
          hypo_pos_meas, dt);
  EXPECT_LT(m_distance, 0.01);
  motion_filter.Predict(dt);
  EXPECT_TRUE(motion_filter.state().isApprox(state_data.car().state()));
  EXPECT_TRUE(
      motion_filter.state_covariance().isApprox(state_data.car().state_cov()));

  QLOG(INFO) << "x_predict: \n" << motion_filter.state();
  QLOG(INFO) << "P_predict: \n" << motion_filter.state_covariance();

  PositionMeasurement pos_meas(0.11, 0.05);
  EXPECT_LE(motion_filter.ComputeMahalanobisDistance(pos_meas), 1.0);
  s = motion_filter.Update(pos_meas);
  QLOG(INFO) << "S_update: \n" << motion_filter.state();
  QLOG(INFO) << "P_update: \n" << motion_filter.state_covariance();
  EXPECT_FALSE(motion_filter.state().isApprox(state_data.car().state()));
  EXPECT_FALSE(
      motion_filter.state_covariance().isApprox(state_data.car().state_cov()));
}

TEST(MotionFilter, Constructor) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  params.set_type(MotionFilterParamProto::CAR_CV);
  std::vector<MotionFilterParamProto> v{params, params};
  IMMProto proto = ToImmProto(v, {0.5, 0.5});
  MotionFilter<CarState> motion_filter(proto);
  EXPECT_FALSE(motion_filter.DebugIsEnabled());
  MotionFilter<CarState> mf1(proto, false);
  EXPECT_FALSE(mf1.DebugIsEnabled());
  MotionFilter<CarState> mf2(proto, true);
  EXPECT_TRUE(mf2.DebugIsEnabled());
}

}  // namespace qcraft::tracker
