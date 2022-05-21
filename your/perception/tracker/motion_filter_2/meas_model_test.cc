#include "onboard/perception/tracker/motion_filter_2/meas_model.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model_type_traits.h"
#include "onboard/perception/tracker/motion_filter_2/point_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(Measurement, Img2DMeasurement) {
  Img2DMeasurement z(1.0, 1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(z.x(), 1.0);
  EXPECT_DOUBLE_EQ(z.y(), 1.0);
  EXPECT_DOUBLE_EQ(z.a(), 1.0);
  EXPECT_DOUBLE_EQ(z.h(), 1.0);

  Img2DMeasurement z1;
  EXPECT_DOUBLE_EQ(z1.x(), 0.0);
  EXPECT_DOUBLE_EQ(z1.y(), 0.0);
  EXPECT_DOUBLE_EQ(z1.a(), 0.0);
  EXPECT_DOUBLE_EQ(z1.h(), 0.0);

  auto z2 = Img2DMeasurement(1.0, 1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(z.x(), 1.0);
  EXPECT_DOUBLE_EQ(z.y(), 1.0);
  EXPECT_DOUBLE_EQ(z.a(), 1.0);
  EXPECT_DOUBLE_EQ(z.h(), 1.0);

  z.x() = 2.0;
  z.y() = 2.0;
  z.a() = 2.0;
  z.h() = 2.0;
  EXPECT_DOUBLE_EQ(z.x(), 2.0);
  EXPECT_DOUBLE_EQ(z.y(), 2.0);
  EXPECT_DOUBLE_EQ(z.a(), 2.0);
  EXPECT_DOUBLE_EQ(z.h(), 2.0);
}

TEST(Img2DMeasurement, Img2DMeasModel) {
  Img2DMeasurement z(1.0, 1.0, 1.0, 1.0);

  Img2DState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.a() = 3.0;
  state.h() = 4.0;
  using MeasModelType = Img2DMeasModel<Img2DState>;
  MeasModelType model;

  QLOG(INFO) << "\n" << model.ComputeH();

  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.x(), 1.0);
  EXPECT_DOUBLE_EQ(hx.y(), 2.0);
  EXPECT_DOUBLE_EQ(hx.a(), 3.0);
  EXPECT_DOUBLE_EQ(hx.h(), 4.0);

  QLOG(INFO) << hx.size();

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/img_2d_model.pb.txt",
      &params));

  Covariance<Img2DMeasurement> R;
  Img2DState s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  QCHECK(model.GetMeasurementNoise(params, &R, s));
  using M = Img2DMeasurement;
  EXPECT_DOUBLE_EQ(R(M::X, M::X), Sqr(0.05 * 1.0));
  EXPECT_DOUBLE_EQ(R(M::Y, M::Y), Sqr(0.05 * 1.0));
  EXPECT_DOUBLE_EQ(R(M::A, M::A), Sqr(1e-1));
  EXPECT_DOUBLE_EQ(R(M::H, M::H), Sqr(0.05 * 1.0));
}

TEST(FunctionTest, PosSpeedMeasModel) {
  auto z = PosSpeedMeasurement(1.0, 1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(z.x(), 1.0);
  EXPECT_DOUBLE_EQ(z.y(), 1.0);
  EXPECT_DOUBLE_EQ(z.vx(), 1.0);
  EXPECT_DOUBLE_EQ(z.vy(), 1.0);

  PointState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.vx() = 3.0;
  state.vy() = 4.0;
  using MeasModelType =
      typename MeasModelTypeSelector<PointState, PosSpeedMeasurement>::type;
  MeasModelType model;
  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.x(), 1.0);
  EXPECT_DOUBLE_EQ(hx.y(), 2.0);
  EXPECT_DOUBLE_EQ(hx.vx(), 3.0);
  EXPECT_DOUBLE_EQ(hx.vy(), 4.0);

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));

  Covariance<PosSpeedMeasurement> R;
  QCHECK(model.GetMeasurementNoise(params, &R));
  using M = PosSpeedMeasurement;
  EXPECT_DOUBLE_EQ(R(M::X, M::X), Sqr(0.5));
  EXPECT_DOUBLE_EQ(R(M::Y, M::Y), Sqr(0.5));
  EXPECT_DOUBLE_EQ(R(M::VEL_X, M::VEL_X), Sqr(2.0));
  EXPECT_DOUBLE_EQ(R(M::VEL_Y, M::VEL_Y), Sqr(2.0));
}

TEST(MeasModelTypeSelector, TypeTest) {
  PosSpeedMeasurement z(1.0, 1.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(z.x(), 1.0);
  EXPECT_DOUBLE_EQ(z.y(), 1.0);
  EXPECT_DOUBLE_EQ(z.vx(), 1.0);
  EXPECT_DOUBLE_EQ(z.vy(), 1.0);

  PointState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.vx() = 3.0;
  state.vy() = 4.0;
  using MeasModelType =
      MeasModelTypeSelector<PointState, PosSpeedMeasurement>::type;
  MeasModelType model;
  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.x(), 1.0);
  EXPECT_DOUBLE_EQ(hx.y(), 2.0);
  EXPECT_DOUBLE_EQ(hx.vx(), 3.0);
  EXPECT_DOUBLE_EQ(hx.vy(), 4.0);

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params));

  Covariance<PosSpeedMeasurement> R;
  QCHECK(model.GetMeasurementNoise(params, &R));
  using M = PosSpeedMeasurement;
  EXPECT_DOUBLE_EQ(R(M::X, M::X), Sqr(0.5));
  EXPECT_DOUBLE_EQ(R(M::Y, M::Y), Sqr(0.5));
  EXPECT_DOUBLE_EQ(R(M::VEL_X, M::VEL_X), Sqr(2.0));
  EXPECT_DOUBLE_EQ(R(M::VEL_Y, M::VEL_Y), Sqr(2.0));
}

TEST(Measurement, PositionMeasurement) {
  PositionMeasurement z(1.0, 1.0);
  EXPECT_DOUBLE_EQ(z.x(), 1.0);
  EXPECT_DOUBLE_EQ(z.y(), 1.0);

  PositionMeasurement z1;
  EXPECT_DOUBLE_EQ(z1.x(), 0.0);
  EXPECT_DOUBLE_EQ(z1.y(), 0.0);

  auto z2 = PositionMeasurement(1.0, 1.0);
  EXPECT_DOUBLE_EQ(z2.x(), 1.0);
  EXPECT_DOUBLE_EQ(z2.y(), 1.0);

  z.x() = 2.0;
  z.y() = 2.0;
  EXPECT_DOUBLE_EQ(z.x(), 2.0);
  EXPECT_DOUBLE_EQ(z.y(), 2.0);
}

TEST(MeasModelTypeSelector, PosMeasModel) {
  PositionMeasurement z(1.0, 1.0);

  PointState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.vx() = 3.0;
  state.vy() = 4.0;
  using MeasModelType =
      MeasModelTypeSelector<PointState, PositionMeasurement>::type;
  MeasModelType model;

  QLOG(INFO) << "\n" << model.ComputeH();

  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.x(), 1.0);
  EXPECT_DOUBLE_EQ(hx.y(), 2.0);
  QLOG(INFO) << hx.size();

  MotionFilterParamProto params_point;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params_point));

  Covariance<PositionMeasurement> R;
  QCHECK(model.GetMeasurementNoise(params_point, &R));
  using M = PositionMeasurement;
  EXPECT_DOUBLE_EQ(R(M::X, M::X), Sqr(0.5));
  EXPECT_DOUBLE_EQ(R(M::Y, M::Y), Sqr(0.5));

  CarState state2;
  state2.x() = 1.0;
  state2.y() = 2.0;
  using MeasModelType2 =
      MeasModelTypeSelector<CarState, PositionMeasurement>::type;
  MeasModelType2 model2;

  QLOG(INFO) << "\n" << model2.ComputeH();

  auto hx2 = model2.ComputeExpectedMeasurement(state2);
  EXPECT_DOUBLE_EQ(hx2.x(), 1.0);
  EXPECT_DOUBLE_EQ(hx2.y(), 2.0);
  QLOG(INFO) << hx2.size();

  MotionFilterParamProto params_car;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params_car));

  QCHECK(model2.GetMeasurementNoise(params_car, &R));
  EXPECT_DOUBLE_EQ(R(M::X, M::X), Sqr(1.0));
  EXPECT_DOUBLE_EQ(R(M::Y, M::Y), Sqr(1.0));
}

TEST(Measurement, HeadingMeasurement) {
  HeadingMeasurement z(1.0);
  EXPECT_DOUBLE_EQ(z.yaw(), 1.0);

  HeadingMeasurement z1;
  EXPECT_DOUBLE_EQ(z1.yaw(), 0.0);

  auto z2 = HeadingMeasurement(1.0);
  EXPECT_DOUBLE_EQ(z2.yaw(), 1.0);

  z.yaw() = 2.0;
  EXPECT_DOUBLE_EQ(z.yaw(), 2.0);
}

TEST(MeasModelTypeSelector, HeadingMeasModel) {
  HeadingMeasurement z(1.0);

  CarState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.yaw() = 3.0;
  using MeasModelType =
      MeasModelTypeSelector<CarState, HeadingMeasurement>::type;
  MeasModelType model;

  QLOG(INFO) << "\n" << model.ComputeH();

  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.yaw(), 3.0);
  QLOG(INFO) << hx.size();

  MotionFilterParamProto params_car;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params_car));

  Covariance<HeadingMeasurement> R;
  using M = HeadingMeasurement;
  QCHECK(model.GetMeasurementNoise(params_car, &R));
  EXPECT_DOUBLE_EQ(R(M::YAW, M::YAW), Sqr(d2r(5.0)));
}

TEST(Measurement, SpeedMeasurement) {
  SpeedMeasurement z(1.0, 2.0);
  EXPECT_DOUBLE_EQ(z.vx(), 1.0);
  EXPECT_DOUBLE_EQ(z.vy(), 2.0);

  SpeedMeasurement z1;
  EXPECT_DOUBLE_EQ(z1.vx(), 0.0);
  EXPECT_DOUBLE_EQ(z1.vy(), 0.0);

  auto z2 = SpeedMeasurement(1.0, 2.0);
  EXPECT_DOUBLE_EQ(z2.vx(), 1.0);
  EXPECT_DOUBLE_EQ(z2.vy(), 2.0);

  z.vx() = 3.0;
  z.vy() = 4.0;
  EXPECT_DOUBLE_EQ(z.vx(), 3.0);
  EXPECT_DOUBLE_EQ(z.vy(), 4.0);
}

TEST(MeasModelTypeSelector, SpeedMeasModel) {
  SpeedMeasurement z(1.0, 2.0);

  PointState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.vx() = 3.0;
  state.vy() = 4.0;
  using MeasModelType =
      MeasModelTypeSelector<PointState, SpeedMeasurement>::type;
  MeasModelType model;

  QLOG(INFO) << "\n" << model.ComputeH();

  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.vx(), 3.0);
  EXPECT_DOUBLE_EQ(hx.vy(), 4.0);
  QLOG(INFO) << hx.size();

  MotionFilterParamProto params_point;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/point_model.pb.txt",
      &params_point));

  Covariance<SpeedMeasurement> R;
  using M = SpeedMeasurement;
  QCHECK(model.GetMeasurementNoise(params_point, &R));
  EXPECT_DOUBLE_EQ(R(M::VEL_X, M::VEL_X), Sqr(2.0));
  EXPECT_DOUBLE_EQ(R(M::VEL_Y, M::VEL_Y), Sqr(2.0));
}

TEST(Measurement, VelocityMeasurement) {
  VelocityMeasurement z(1.0);
  EXPECT_DOUBLE_EQ(z.vel(), 1.0);

  VelocityMeasurement z1;
  EXPECT_DOUBLE_EQ(z1.vel(), 0.0);

  auto z2 = VelocityMeasurement(1.0);
  EXPECT_DOUBLE_EQ(z2.vel(), 1.0);

  z.vel() = 2.0;
  EXPECT_DOUBLE_EQ(z.vel(), 2.0);
}

TEST(MeasModelTypeSelector, VelocityMeasModel) {
  VelocityMeasurement z(1.0);

  CarState state;
  state.x() = 1.0;
  state.y() = 2.0;
  state.vel() = 3.0;
  using MeasModelType =
      MeasModelTypeSelector<CarState, VelocityMeasurement>::type;
  MeasModelType model;

  QLOG(INFO) << "\n" << model.ComputeH();

  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.vel(), 3.0);
  QLOG(INFO) << hx.size();

  MotionFilterParamProto params_car;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params_car));

  Covariance<VelocityMeasurement> R;
  using M = VelocityMeasurement;
  QCHECK(model.GetMeasurementNoise(params_car, &R));
  EXPECT_DOUBLE_EQ(R(M::VEL, M::VEL), Sqr(2.0));
}

}  // namespace qcraft::tracker
