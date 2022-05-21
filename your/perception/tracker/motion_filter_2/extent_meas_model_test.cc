#include "onboard/perception/tracker/motion_filter_2/extent_meas_model.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model_type_traits.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(FunctionTest, BBoxMeasModel) {
  auto z = BBoxMeasurement(1.0, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(z.length(), 1.0);
  EXPECT_DOUBLE_EQ(z.width(), 2.0);
  EXPECT_DOUBLE_EQ(z.height(), 3.0);

  auto z1 = BBoxMeasurement(1.0, 2.0);
  EXPECT_DOUBLE_EQ(z1.length(), 1.0);
  EXPECT_DOUBLE_EQ(z1.width(), 2.0);
  EXPECT_DOUBLE_EQ(z1.height(), 0.0);

  BBoxState state;
  state.length() = 11.0;
  state.width() = 12.0;
  state.height() = 13.0;
  using MeasModelType =
      typename MeasModelTypeSelector<BBoxState, BBoxMeasurement>::type;
  MeasModelType model;
  auto hx = model.ComputeExpectedMeasurement(state);
  EXPECT_DOUBLE_EQ(hx.length(), 11.0);
  EXPECT_DOUBLE_EQ(hx.width(), 12.0);
  EXPECT_DOUBLE_EQ(hx.height(), 13.0);

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  Covariance<BBoxMeasurement> R;
  QCHECK(model.GetMeasurementNoise(params, &R));
  using M = BBoxMeasurement;
  EXPECT_DOUBLE_EQ(R(M::LENGTH, M::LENGTH),
                   Sqr(params.state_length_measurement_noise_std()));
  EXPECT_DOUBLE_EQ(R(M::WIDTH, M::WIDTH),
                   Sqr(params.state_width_measurement_noise_std()));
  EXPECT_DOUBLE_EQ(R(M::HEIGHT, M::HEIGHT),
                   Sqr(params.state_height_measurement_noise_std()));
}

}  // namespace qcraft::tracker
