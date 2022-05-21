#include "onboard/control/longitudinal_postprocess/lon_postprocess.h"

#include <iostream>
#include <memory>

#include "gtest/gtest.h"
#include "onboard/utils/proto_util.h"

qcraft::ControllerConf control_config_;
qcraft::VehicleDriveParamsProto vehicle_drive_params_;
namespace qcraft {
void InitTest() {
  vehicle_drive_params_.set_enable_calibration_v2(true);
  vehicle_drive_params_.set_throttle_deadzone(16.0);
  vehicle_drive_params_.set_brake_deadzone(20.0);
  auto idle_v_a_plf = vehicle_drive_params_.mutable_calibration_table_v2()
                          ->mutable_idle_v_a_plf();
  auto a_throttle_plf = vehicle_drive_params_.mutable_calibration_table_v2()
                            ->mutable_a_throttle_plf();
  auto a_brake_plf = vehicle_drive_params_.mutable_calibration_table_v2()
                         ->mutable_a_brake_plf();
  TextToProto(
      "x : -3 x : -1 x : 0 x : 1 x : 1.8 x : 5 x : 10 x : 15 y : 0.5 y : -0.2 "
      "y : 0.4 y : 0.2 y : 0.0 y : -0.2 y : -0.4 y : -0.7",
      idle_v_a_plf);

  TextToProto(
      "x : -3 x : 0 x : 1 x : 2 x : 3y : 50 y : 16 y : 20 y : "
      "40 y : 50",
      a_throttle_plf);

  TextToProto("x : -4 x : -1 x : 0 x : 4 y : 40 y : 30 y : 20 y : 60}",
              a_brake_plf);

  control_config_.set_ebrake_enable(false);
  control_config_.mutable_full_stop_condition()
      ->set_abs_linear_speed_upperlimit(1.0);
  control_config_.mutable_full_stop_condition()
      ->set_abs_planner_speed_upperlimit(0.1);
  control_config_.mutable_full_stop_condition()->set_standstill_acceleration(
      -0.4);
  control_config_.mutable_full_stop_condition()->set_lockdown_acceleration(
      -1.0);
  control_config_.set_acceleration_interface(
      qcraft::AccelerationInterface::CLOSEDLOOP);
  control_config_.set_control_period(0.01);
}

TEST(LonPostProcess, NonClosedAccThrottle) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;
  auto manager = std::make_unique<qcraft::control::LonPostProcess>(
      &control_config_, &vehicle_drive_params_);

  input.gear_fb = qcraft::Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.is_standstill = false;
  input.pitch_pose = 0.0;
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 1.0;
  input.acc_feedback = 0.9;
  input.acc_target = 1.5;
  input.speed_feedback = 5.0;
  input.speed_planner = 3.0;
  manager->Process(input, &control_cmd, &control_debug);

  constexpr double kEpsilon = 1e-5;
  EXPECT_NEAR(control_cmd.acceleration(), 1.5, kEpsilon);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_idle(), -0.2,
              kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_pure(), 1.7,
              kEpsilon);
  EXPECT_NEAR(control_cmd.throttle(), 34.0, kEpsilon);

  input.gear_fb = qcraft::Chassis::GEAR_REVERSE;
  input.is_auto_mode = true;
  input.is_standstill = false;
  input.pitch_pose = 0.0;
  input.steer_wheel_angle = 0.0;
  input.acc_planner = -0.5;
  input.acc_feedback = -0.9;
  input.acc_target = -0.8;
  input.speed_feedback = -1.0;
  input.speed_planner = -0.5;
  manager->Process(input, &control_cmd, &control_debug);

  EXPECT_NEAR(control_cmd.acceleration(), -0.8, kEpsilon);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_idle(), -0.2,
              kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_pure(), -0.6,
              kEpsilon);
  EXPECT_NEAR(control_cmd.throttle(), 22.8, kEpsilon);
}

TEST(LonPostProcess, NonClosedAccStandStill1) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;
  auto manager = std::make_unique<qcraft::control::LonPostProcess>(
      &control_config_, &vehicle_drive_params_);

  input.gear_fb = qcraft::Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.is_standstill = false;
  input.pitch_pose = 0.1;  // down slope, offset < 0
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.0;
  input.acc_feedback = 0.0;
  input.acc_target = 0.0;
  input.speed_feedback = 0.5;
  input.speed_planner = 0.0;

  constexpr double kEpsilon = 1e-3;
  for (int i = 0; i < 200; ++i) {
    manager->Process(input, &control_cmd, &control_debug);
    EXPECT_NEAR(control_cmd.acceleration_offset(), -0.979031, kEpsilon);
    EXPECT_EQ(control_cmd.debug().simple_mpc_debug().is_full_stop(), true);
    const auto acc_calibration = control_cmd.acceleration_calibration();
    const auto exp_acc = std::max(
        input.acc_target - 0.01 * (i + 1),
        control_config_.full_stop_condition().standstill_acceleration() -
            std::fabs(control_cmd.acceleration_offset()));
    EXPECT_NEAR(acc_calibration, exp_acc, kEpsilon);
  }
}

TEST(LonPostProcess, NonClosedAccStandStill2) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;
  auto manager = std::make_unique<qcraft::control::LonPostProcess>(
      &control_config_, &vehicle_drive_params_);

  input.gear_fb = qcraft::Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.is_standstill = false;
  input.pitch_pose = 0.1;  // down slope, offset < 0
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.0;
  input.acc_feedback = 0.0;
  input.acc_target = 0.0;
  input.speed_feedback = 0.005;
  input.speed_planner = 0.0;

  constexpr double kEpsilon = 1e-3;
  for (int i = 0; i < 200; ++i) {
    manager->Process(input, &control_cmd, &control_debug);
    EXPECT_NEAR(control_cmd.acceleration_offset(), -0.979031, kEpsilon);
    EXPECT_EQ(control_cmd.debug().simple_mpc_debug().is_full_stop(), true);
    const auto acc_calibration = control_cmd.acceleration_calibration();
    const auto exp_acc =
        std::max(input.acc_target - 0.01 * 0.4 * (i + 1),
                 control_config_.full_stop_condition().lockdown_acceleration() -
                     std::fabs(control_cmd.acceleration_offset()));
    EXPECT_NEAR(acc_calibration, exp_acc, kEpsilon);
  }
}

TEST(LonPostProcess, NonClosedParking) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;
  auto manager = std::make_unique<qcraft::control::LonPostProcess>(
      &control_config_, &vehicle_drive_params_);

  input.gear_fb = qcraft::Chassis::GEAR_REVERSE;
  input.is_auto_mode = true;
  input.is_standstill = false;
  input.pitch_pose = 0.0;
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.5;
  input.acc_feedback = 0.5;
  input.acc_target = 0.5;
  input.speed_feedback = -1.0;
  input.speed_planner = -1.0;
  manager->Process(input, &control_cmd, &control_debug);

  constexpr double kEpsilon = 1e-5;
  EXPECT_NEAR(control_cmd.acceleration(), 0.5, kEpsilon);
  EXPECT_NEAR(control_cmd.acceleration_offset(), 0.0, kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_idle(), -0.2,
              kEpsilon);
  EXPECT_NEAR(control_debug.calibration_debug_proto().acceleration_pure(), 0.7,
              kEpsilon);
  EXPECT_NEAR(control_cmd.brake(), 27.0, kEpsilon);
}

TEST(LonPostProcess, ClosedAccThrottle) {
  InitTest();
  qcraft::control::LonPostProcessInput input;
  qcraft::ControlCommand control_cmd;
  qcraft::ControllerDebugProto control_debug;

  control_config_.set_enable_speed_mode_manager(true);
  control_config_.mutable_closed_loop_acc_conf()->set_enable_closed_loop_acc(
      true);
  auto manager = std::make_unique<qcraft::control::LonPostProcess>(
      &control_config_, &vehicle_drive_params_);

  input.gear_fb = qcraft::Chassis::GEAR_DRIVE;
  input.is_auto_mode = true;
  input.is_standstill = false;
  input.pitch_pose = 0.1;  // down slope, offset < 0
  input.steer_wheel_angle = 0.0;
  input.acc_planner = 0.0;
  input.acc_feedback = 0.0;
  input.acc_target = 0.0;
  input.speed_feedback = 0.5;
  input.speed_planner = 0.0;

  constexpr double kEpsilon = 1e-3;
  for (int i = 0; i < 200; ++i) {
    manager->Process(input, &control_cmd, &control_debug);
    EXPECT_EQ(control_cmd.debug().simple_mpc_debug().is_full_stop(), true);
    EXPECT_NEAR(control_cmd.acceleration_calibration(),
                control_debug.speed_mode_debug_proto().input_acc(), kEpsilon);
  }
}

}  // namespace qcraft
