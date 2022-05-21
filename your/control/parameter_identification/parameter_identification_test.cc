#include "onboard/control/parameter_identification/parameter_identification.h"

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-5;
ParameterIdentificator::SteerBiasIdentificationInputData MakeInputData(
    double front_wheel_angle, double kappa) {
  return {.front_wheel_angle = front_wheel_angle, .kappa = kappa};
}

TEST(ParameterIdentificatorTest, CalculateSteerBias) {
  VehicleDriveParamsProto vehicle_drive_params;
  VehicleGeometryParamsProto vehicle_geometry_params;
  ControllerConf control_conf;
  vehicle_geometry_params.set_wheel_base(4.5);
  ParameterIdentificator parameter_identificator(
      vehicle_drive_params, vehicle_geometry_params, control_conf);

  EXPECT_EQ(parameter_identificator.prev_valid_result_num(), 0);
  EXPECT_NEAR(
      parameter_identificator.CalculateSteerBias(MakeInputData(0.1, 0.01)),
      -0.0550303, kEpsilon);
  EXPECT_EQ(parameter_identificator.prev_valid_result_num(), 1);

  EXPECT_NEAR(
      parameter_identificator.CalculateSteerBias(MakeInputData(0.2, 0.03)),
      -0.0604208, kEpsilon);
  EXPECT_EQ(parameter_identificator.prev_valid_result_num(), 2);
  EXPECT_NEAR(
      parameter_identificator.CalculateSteerBias(MakeInputData(0.4, 0.05)),
      -0.0998424, kEpsilon);
  EXPECT_EQ(parameter_identificator.prev_valid_result_num(), 3);
}

}  // namespace
}  // namespace qcraft::control
