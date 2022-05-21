
#include "onboard/perception/tracker/motion_filter_2/estimator.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(Estimator, CarEstimator) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  params.set_type(MotionFilterParamProto::CAR_CV);
  std::vector<MotionFilterParamProto> v{params, params};
  Estimator estimator(ToImmProto(v, {0.5, 0.5}));
  EXPECT_TRUE(estimator.IsCarModel());
  EXPECT_FALSE(estimator.IsPointModel());

  CarState state;
  state.x() = 1.0;
  state.y() = 1.0;
  state.vel() = 1.0;
  state.yaw() = 0.0;
  state.yawd() = 0.1;
  state.acc() = 0.2;

  auto init_state = StateData(state);

  const double timestamp = 0.0;
  estimator.Init(init_state, timestamp);

  QLOG(INFO) << estimator.GetStateData();
  QLOG(INFO) << "State Pos\n" << estimator.GetStatePos();
  QLOG(INFO) << "State Cov\n" << estimator.GetStatePosCov();

  const double dt = 0.1;
  auto state_data = estimator.ComputePrediction(timestamp + dt);
  estimator.Predict(timestamp + dt);
  EXPECT_TRUE(estimator.GetStateData().point().state().isApprox(
      state_data.point().state()));
  EXPECT_TRUE(estimator.GetStateData().point().state_cov().isApprox(
      state_data.point().state_cov()));
  EXPECT_TRUE(estimator.GetStateData().car().state().isApprox(
      state_data.car().state()));
  EXPECT_TRUE(estimator.GetStateData().car().state_cov().isApprox(
      state_data.car().state_cov()));

  QLOG(INFO) << "Predict:";
  QLOG(INFO) << estimator.GetStateData();

  PositionMeasurement hypo_pos_meas(1.1, 1.1);
  const auto m_distance =
      estimator.ComputeMahalanobisDistanceFromPredictionInMeasurementSpace(
          hypo_pos_meas, timestamp + dt);
  EXPECT_LT(m_distance, 0.01);

  PositionMeasurement pos_meas(0.11, 0.05);
  EXPECT_LE(estimator.ComputeMahalanobisDistance(pos_meas), 1.0);
  estimator.Update(pos_meas);
  QLOG(INFO) << "Update:";
  QLOG(INFO) << estimator.GetStateData();
  EXPECT_FALSE(estimator.GetStateData().point().state().isApprox(
      state_data.point().state()));
  EXPECT_FALSE(estimator.GetStateData().point().state_cov().isApprox(
      state_data.point().state_cov()));
  EXPECT_FALSE(estimator.GetStateData().car().state().isApprox(
      state_data.car().state()));
  EXPECT_FALSE(estimator.GetStateData().car().state_cov().isApprox(
      state_data.car().state_cov()));

  estimator.SetStateData(StateData(state));
  EXPECT_TRUE(estimator.GetStateData().car().state().isApprox(state));
}

TEST(Estimator, PointEstimator) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));
  auto params_ca = params;
  params_ca.set_type(MotionFilterParamProto::POINT_CA);
  auto params_cp = params;
  params_cp.set_type(MotionFilterParamProto::POINT_CP);
  std::vector<MotionFilterParamProto> v{params_cp, params_ca};

  Estimator estimator(ToImmProto(v, {0.5, 0.5}));
  EXPECT_TRUE(estimator.IsPointModel());
  EXPECT_FALSE(estimator.IsCarModel());

  PointState state;
  state << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  auto init_state = StateData(state);

  const double timestamp = 0.0;
  estimator.Init(init_state, timestamp);

  QLOG(INFO) << estimator.GetStateData();

  const double dt = 0.1;
  auto state_data = estimator.ComputePrediction(timestamp + dt);
  estimator.Predict(timestamp + dt);
  EXPECT_TRUE(estimator.GetStateData().point().state().isApprox(
      state_data.point().state()));
  EXPECT_TRUE(estimator.GetStateData().point().state_cov().isApprox(
      state_data.point().state_cov()));
  EXPECT_TRUE(estimator.GetStateData().car().state().isApprox(
      state_data.car().state()));
  EXPECT_TRUE(estimator.GetStateData().car().state_cov().isApprox(
      state_data.car().state_cov()));

  QLOG(INFO) << "Predict:";
  QLOG(INFO) << estimator.GetStateData();

  PosSpeedMeasurement pos_speed(0.11, 0.05, 0, 0);
  PositionMeasurement pos(0.11, 0.05);
  SpeedMeasurement speed(0, 0);
  auto estimator_update_split = estimator;
  estimator.Update(pos_speed);
  estimator_update_split.Update(speed);
  estimator_update_split.Update(pos);
  QLOG(INFO) << "Update:" << estimator.GetStateData();
  QLOG(INFO) << "Update estimator_update_split:"
             << estimator_update_split.GetStateData();
  EXPECT_FALSE(estimator.GetStateData().point().state().isApprox(
      state_data.point().state()));
  EXPECT_FALSE(estimator.GetStateData().point().state_cov().isApprox(
      state_data.point().state_cov()));
  EXPECT_FALSE(estimator.GetStateData().car().state().isApprox(
      state_data.car().state()));
  EXPECT_FALSE(estimator.GetStateData().car().state_cov().isApprox(
      state_data.car().state_cov()));

  EXPECT_TRUE(estimator.GetStateData().point().state().isApprox(
      estimator_update_split.GetStateData().point().state()));
  EXPECT_TRUE(estimator.GetStateData().point().state_cov().isApprox(
      estimator_update_split.GetStateData().point().state_cov()));
}

TEST(Estimator, Function) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  auto params_ca = params;
  params_ca.set_type(MotionFilterParamProto::CAR_CA);
  auto params_ctrv = params;
  params_ctrv.set_type(MotionFilterParamProto::CAR_CTRV);
  std::vector<MotionFilterParamProto> v{params_ca, params_ctrv};
  Estimator estimator(ToImmProto(v, {0.5, 0.5}));

  CarState state;
  state.x() = 1.0;
  state.y() = 1.0;
  state.vel() = 0.0;
  state.yaw() = 1.0;
  state.yawd() = 0.0;
  state.acc() = 0.0;

  const auto init_state = StateData(state);

  const double timestamp = 0.0;
  estimator.Init(init_state, timestamp);
  estimator.Predict(timestamp + 0.1);
  PositionMeasurement pos_meas(1.11, 1.05);
  estimator.Update(pos_meas);

  auto s = estimator.GetStateData();
  s.car().state().x() = 10.0;
  s.car().state().y() = 4.0;
  Eigen::Matrix2d pos_cov;
  pos_cov << 1, 2, 3, 4;
  s.SetPosCov(pos_cov);

  Estimator estimator_init = estimator;
  estimator_init.InitStateAndCov(s, timestamp);
  estimator.SetStateData(s);

  QLOG(INFO) << "State Init\n" << estimator_init.GetStateData();
  QLOG(INFO) << "State Set\n" << estimator.GetStateData();

  EXPECT_TRUE(estimator.GetStateData().point().state().isApprox(
      estimator_init.GetStateData().point().state()));
  EXPECT_TRUE(estimator.GetStateData().point().state_cov().isApprox(
      estimator_init.GetStateData().point().state_cov()));
  EXPECT_TRUE(estimator.GetStateData().car().state().isApprox(
      estimator_init.GetStateData().car().state()));
  EXPECT_TRUE(estimator.GetStateData().car().state_cov().isApprox(
      estimator_init.GetStateData().car().state_cov()));
}

TEST(Estimator, UpdateAllMeasVSUpdateMeasOneByOne) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));
  auto params_ca = params;
  params_ca.set_type(MotionFilterParamProto::POINT_CA);
  auto params_cp = params;
  params_cp.set_type(MotionFilterParamProto::POINT_CP);
  std::vector<MotionFilterParamProto> v{params_cp, params_ca};

  Estimator estimator(ToImmProto(v, {0.5, 0.5}));
  EXPECT_TRUE(estimator.IsPointModel());
  EXPECT_FALSE(estimator.IsCarModel());

  PointState state;
  state << -981.555, 2768.47, 14.4368, 3.69982, 0.0, 0.0;

  auto init_state = StateData(state);

  const double timestamp = 1646212311.0821;
  estimator.Init(init_state, timestamp);

  QLOG(INFO) << estimator.GetStateData();

  const double next_timestamp = 1646212311.1571;
  estimator.Predict(next_timestamp);

  PositionMeasurement pos(-980.76, 2768.69);
  SpeedMeasurement speed(14.5263, 3.19708);
  PosSpeedMeasurement pos_speed(pos.x(), pos.y(), speed.vx(), speed.vy());
  auto estimator_update_split = estimator;
  estimator.Update(pos_speed);
  estimator_update_split.Update(speed);
  estimator_update_split.Update(pos);
  QLOG(INFO) << "Update:" << estimator.GetStateData();
  QLOG(INFO) << "Update estimator_update_split:"
             << estimator_update_split.GetStateData();

  EXPECT_FALSE(estimator.GetStateData().point().state().isApprox(
      estimator_update_split.GetStateData().point().state()));
  EXPECT_FALSE(estimator.GetStateData().point().state_cov().isApprox(
      estimator_update_split.GetStateData().point().state_cov()));
}

TEST(Estimator, ExtentTest) {
  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));
  auto params_ca = params;
  params_ca.set_type(MotionFilterParamProto::CAR_CA);
  auto params_ctrv = params;
  params_ctrv.set_type(MotionFilterParamProto::CAR_CTRV);
  std::vector<MotionFilterParamProto> v{params_ca, params_ctrv};

  Estimator estimator(ToImmProto(v, {0.5, 0.5}));

  CarState state;
  state << -981.555, 2768.47, 0.0, 0.0, 0.0, 0.0;

  auto init_state = StateData(state);

  const double timestamp = 1646212311.0821;
  estimator.Init(init_state, timestamp);

  QLOG(INFO) << estimator.GetStateData();

  const double next_timestamp = 1646212311.1571;
  BBoxState bbox_state(5.0, 2.5, 0.0);
  estimator.InitExtent(bbox_state);
  estimator.PredictExtent(next_timestamp);

  const auto extent = estimator.GetExtentStateData();

  EXPECT_DOUBLE_EQ(extent.state().length(), 5.0);
  EXPECT_DOUBLE_EQ(extent.state().width(), 2.5);
  EXPECT_DOUBLE_EQ(extent.state().height(), 0.0);

  BBoxMeasurement bbox(7.0, 3.0, 0.0);
  estimator.UpdateExtent(bbox);
  QLOG(INFO) << "S_update: \n" << estimator.GetExtentStateData().state();
  QLOG(INFO) << "P_update: \n" << estimator.GetExtentStateData().state_cov();
}

}  // namespace qcraft::tracker
