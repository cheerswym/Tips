#include "onboard/control/controllers/controller_util.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace control {
namespace {

constexpr double kEpsilon = 0.0001;

TEST(ControllerUtilTest, IsCmdBackToZeroTest) {
  std::vector<std::vector<double>> test_set;
  test_set.push_back({1, 2, 3, 4, 5});       // Expect 0
  test_set.push_back({5, 4, 3, 2, 1});       // Expect 1
  test_set.push_back({-5, -4, -3, -2, -1});  // Expect 1
  test_set.push_back({5, -4, 3, -2, 1});     // Expect 0
  test_set.push_back({-5, 4, -3, 2, -1});    // Expect 0
  test_set.push_back({1, 1, 1, 1, 0});       // Expect 1
  test_set.push_back({1, 1, 1, 1, 2});       // Expect 0

  std::vector<bool> result;
  result.reserve(test_set.size());
  for (const auto& test : test_set) {
    result.push_back(IsCmdBackToZero(test));
  }

  EXPECT_FALSE(result[0]);
  EXPECT_TRUE(result[1]);
  EXPECT_TRUE(result[2]);
  EXPECT_FALSE(result[3]);
  EXPECT_FALSE(result[4]);
  EXPECT_TRUE(result[5]);
  EXPECT_FALSE(result[6]);
}

TEST(ControllerUtilTest, Kappa2SteerPercentageTest) {
  const double kappa = 0.1;
  const double wheel_base = 2.84;
  VehicleDriveParamsProto vehicle_drive_params;
  vehicle_drive_params.set_max_steer_angle(8.2);
  vehicle_drive_params.set_steer_ratio(16.0);
  EXPECT_NEAR(Kappa2SteerPercentage(kappa, wheel_base,
                                    vehicle_drive_params.steer_ratio(),
                                    vehicle_drive_params.max_steer_angle()),
              53.993, kEpsilon);
}

TEST(ControllerUtilTest, SteerRateTest) {
  const double wheel_base = 2.84;
  const double dt = 0.01;
  VehicleDriveParamsProto vehicle_drive_params;
  vehicle_drive_params.set_max_steer_angle(8.2);
  vehicle_drive_params.set_steer_ratio(16.0);

  const double kappa_a_0 = 0.07;
  const double kappa_a_1 = 0.08;
  const double kappa_a_rate = (kappa_a_1 - kappa_a_0) / dt;
  const double steer_angle_a_0 = Kappa2SteerAngle(
      kappa_a_0, wheel_base, vehicle_drive_params.steer_ratio());
  const double steer_angle_a_1 = Kappa2SteerAngle(
      kappa_a_1, wheel_base, vehicle_drive_params.steer_ratio());
  const double steer_angle_rate_a = (steer_angle_a_1 - steer_angle_a_0) / dt;
  const double steer_angle_rate_calc_a =
      KappaRate2SteerRate(kappa_a_rate, 0.5 * (kappa_a_0 + kappa_a_1),
                          wheel_base, vehicle_drive_params.steer_ratio());
  EXPECT_NEAR(steer_angle_rate_calc_a / steer_angle_rate_a, 1.0, kEpsilon)
      << "steer_angle_rate_calc = " << steer_angle_rate_calc_a
      << ", steer_angle_rate = " << steer_angle_rate_a;

  const double kappa_b_0 = 0.002;
  const double kappa_b_1 = -0.006;
  const double kappa_b_rate = (kappa_b_1 - kappa_b_0) / dt;
  const double steer_angle_b_0 = Kappa2SteerAngle(
      kappa_b_0, wheel_base, vehicle_drive_params.steer_ratio());
  const double steer_angle_b_1 = Kappa2SteerAngle(
      kappa_b_1, wheel_base, vehicle_drive_params.steer_ratio());
  const double steer_angle_rate_b = (steer_angle_b_1 - steer_angle_b_0) / dt;
  const double steer_angle_rate_calc_b =
      KappaRate2SteerRate(kappa_b_rate, 0.5 * (kappa_b_0 + kappa_b_1),
                          wheel_base, vehicle_drive_params.steer_ratio());
  EXPECT_NEAR(steer_angle_rate_calc_b / steer_angle_rate_b, 1.0, kEpsilon)
      << "steer_angle_rate_calc = " << steer_angle_rate_calc_b
      << ", steer_angle_rate = " << steer_angle_rate_b;
}

TEST(ControllerUtilTest, KappaRateTest) {
  const double wheel_base = 2.84;
  const double steer_ratio = 22.0;
  const double dt = 0.01;

  const double delta_a_0 = 0.01;
  const double delta_a_1 = 0.02;
  const double kappa_a_0 = FrontWheelAngle2Kappa(delta_a_0, wheel_base);
  const double kappa_a_1 = FrontWheelAngle2Kappa(delta_a_1, wheel_base);
  const double delta_rate_a = (delta_a_1 - delta_a_0) / dt;
  const double kappa_rate_expected_a = (kappa_a_1 - kappa_a_0) / dt;
  const double kappa_rate_calc_a = FrontWheelSteerRate2KappaRateWithKappa(
      delta_rate_a, 0.5 * (kappa_a_0 + kappa_a_1), wheel_base);

  EXPECT_NEAR(kappa_rate_calc_a / kappa_rate_expected_a, 1.0, kEpsilon)
      << " kappa_rate_calc = " << kappa_rate_calc_a
      << ", kappa_rate_expected = " << kappa_rate_expected_a;

  const double delta_b_0 = -0.01;
  const double delta_b_1 = 0.02;
  const double kappa_b_0 = FrontWheelAngle2Kappa(delta_b_0, wheel_base);
  const double kappa_b_1 = FrontWheelAngle2Kappa(delta_b_1, wheel_base);
  const double delta_rate_b = (delta_b_1 - delta_b_0) / dt;
  const double kappa_rate_expected_b = (kappa_b_1 - kappa_b_0) / dt;
  const double kappa_rate_calc_b = FrontWheelSteerRate2KappaRateWithKappa(
      delta_rate_b, 0.5 * (kappa_b_0 + kappa_b_1), wheel_base);

  EXPECT_NEAR(kappa_rate_calc_b / kappa_rate_expected_b, 1.0, kEpsilon)
      << " kappa_rate_calc = " << kappa_rate_calc_b
      << ", kappa_rate_expected = " << kappa_rate_expected_b;

  const double steering_speed = delta_rate_b * steer_ratio;
  const double kappa_rate_calc_c = SteeringSpeed2KappaRateWithKappa(
      steering_speed, 0.5 * (kappa_b_0 + kappa_b_1), wheel_base, steer_ratio);
  EXPECT_NEAR(kappa_rate_calc_c / kappa_rate_expected_b, 1.0, kEpsilon)
      << " kappa_rate_calc = " << kappa_rate_calc_b
      << ", kappa_rate_expected = " << kappa_rate_expected_b;
}

}  // namespace
}  // namespace control
}  // namespace qcraft
