#include "onboard/control/calibration/calibration_manager.h"

#include "gtest/gtest.h"

using qcraft::Chassis;
using qcraft::VehicleDriveParamsProto;
using qcraft::control::CalibrationCmd;
using qcraft::control::CalibrationManager;
using qcraft::control::CalibrationValue;
using qcraft::control::ControlCalibrationTable;
using qcraft::control::ControlCalibrationVersion2;

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-5;

TEST(CalibrationManager, Version_1) {
  VehicleDriveParamsProto vehicle_drive_param;
  vehicle_drive_param.set_enable_calibration_v2(false);
  vehicle_drive_param.set_throttle_deadzone(16.0);
  vehicle_drive_param.set_brake_deadzone(20.0);
  const auto calibration_table =
      vehicle_drive_param.mutable_calibration_table();

  auto calibration = calibration_table->add_calibration();
  calibration->set_speed(0.0);
  calibration->set_acceleration(0.0);
  calibration->set_command(14.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(0.0);
  calibration->set_acceleration(1.0);
  calibration->set_command(20.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(10.0);
  calibration->set_acceleration(0.0);
  calibration->set_command(20.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(10.0);
  calibration->set_acceleration(1.0);
  calibration->set_command(40.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(0.0);
  calibration->set_acceleration(-1.0);
  calibration->set_command(-16.0);

  calibration = calibration_table->add_calibration();
  calibration->set_speed(10.0);
  calibration->set_acceleration(-1.0);
  calibration->set_command(-30.0);

  const bool enable_ebrake = false;

  const auto lon_calibration = std::make_shared<CalibrationManager>();
  const auto inti = lon_calibration->Init(vehicle_drive_param, enable_ebrake);
  EXPECT_EQ(inti, absl::OkStatus());

  CalibrationValue calibration_value;
  lon_calibration->UpdateCalibrationValue(0.0, 0.0, Chassis::GEAR_DRIVE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, 14.0, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  CalibrationCmd calibration_cmd;
  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  lon_calibration->UpdateCalibrationValue(5.0, 0.5, Chassis::GEAR_DRIVE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, 23.5, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 23.5, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  lon_calibration->UpdateCalibrationValue(5.0, -0.5, Chassis::GEAR_DRIVE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, -3.0, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);
}

TEST(CalibrationManager, Version_2) {
  VehicleDriveParamsProto vehicle_drive_param;
  vehicle_drive_param.set_enable_calibration_v2(true);
  vehicle_drive_param.set_throttle_deadzone(16.0);
  vehicle_drive_param.set_brake_deadzone(20.0);
  const auto calibration_table_v2 =
      vehicle_drive_param.mutable_calibration_table_v2();

  calibration_table_v2->mutable_idle_v_a_plf()->add_x(-3.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.5);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(0.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.4);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(1.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.2);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(1.8);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(0.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(5.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(-0.2);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(10.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(-0.4);
  calibration_table_v2->mutable_idle_v_a_plf()->add_x(15.0);
  calibration_table_v2->mutable_idle_v_a_plf()->add_y(-0.7);

  calibration_table_v2->mutable_a_throttle_plf()->add_x(-3.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(50.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(0.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(16.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(1.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(20.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(2.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(40.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_x(3.0);
  calibration_table_v2->mutable_a_throttle_plf()->add_y(50.0);

  calibration_table_v2->mutable_a_brake_plf()->add_x(-4.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(40.0);
  calibration_table_v2->mutable_a_brake_plf()->add_x(-1.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(30.0);
  calibration_table_v2->mutable_a_brake_plf()->add_x(0.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(20.0);
  calibration_table_v2->mutable_a_brake_plf()->add_x(4.0);
  calibration_table_v2->mutable_a_brake_plf()->add_y(60.0);

  const bool enable_ebrake = false;

  const auto lon_calibration = std::make_shared<CalibrationManager>();
  const auto inti = lon_calibration->Init(vehicle_drive_param, enable_ebrake);
  EXPECT_EQ(inti, absl::OkStatus());

  CalibrationValue calibration_value;
  lon_calibration->UpdateCalibrationValue(0.0, 0.0, Chassis::GEAR_DRIVE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, -23.98, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  CalibrationCmd calibration_cmd;
  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 23.98, kEpsilon);

  lon_calibration->UpdateCalibrationValue(5.0, 0.5, Chassis::GEAR_DRIVE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, 18.8, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 18.8, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  lon_calibration->UpdateCalibrationValue(5.0, -1.2, Chassis::GEAR_DRIVE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, -30.0, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);

  lon_calibration->UpdateCalibrationValue(-3.0, -1.0, Chassis::GEAR_REVERSE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, 33.0, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 33.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 0.0, kEpsilon);

  lon_calibration->UpdateCalibrationValue(-3.0, 1.0, Chassis::GEAR_REVERSE,
                                          &calibration_value);
  EXPECT_NEAR(calibration_value.control_calibration_value, -25.0, kEpsilon);
  EXPECT_NEAR(calibration_value.ebrake_calibration_value, 0.0, kEpsilon);

  lon_calibration->ComputeLongitudinalCmd(
      calibration_value.control_calibration_value, &calibration_cmd);
  EXPECT_NEAR(calibration_cmd.throttle_cmd, 0.0, kEpsilon);
  EXPECT_NEAR(calibration_cmd.brake_cmd, 25.0, kEpsilon);
}

}  // namespace
}  // namespace qcraft::control
