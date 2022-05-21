#include "onboard/control/control_check/wire_control_check.h"

#include <iostream>
#include <memory>

#include "gtest/gtest.h"

TEST(WireControlCheck, throttle) {
  const auto control_period = 0.01;
  const auto steer_delay = 0.5;
  auto check_ptr = std::make_unique<qcraft::control::WireControlChecker>(
      control_period, steer_delay);

  double acceleration_fb = 0.0;
  double acceleration_cmd = 0.0;
  double kappa_fb = 0.0;
  double kappa_cmd = 0.0;
  double speed = 2.1;
  qcraft::Chassis::GearPosition gear_fb = qcraft::Chassis::GEAR_DRIVE;
  qcraft::AutonomyStateProto_State autonomy_state =
      qcraft::AutonomyStateProto::AUTO_DRIVE;

  bool status = false;
  for (int i = 0; i < 200; ++i) {
    if (i < 50) {
      acceleration_fb = 0.0;
      acceleration_cmd = 0.01 * i;  // 0.5m/s2
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, false);
    } else if (i < 100) {
      acceleration_fb = 0.03 * (i - 50);  // 1.5m/s2
      acceleration_cmd = 0.01 * i;        // 1.0m/s2
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, false);
    } else if (i < 143) {
      acceleration_fb = 0.03 * (i - 50);
      acceleration_cmd = 0.01 * i;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, false);
    } else if (i < 150) {
      acceleration_fb = 0.03 * (i - 50);
      acceleration_cmd = 0.01 * i;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, true);
    } else {
      acceleration_fb = 2.0;   // 3.0m/s2
      acceleration_cmd = 1.5;  // 1.5m/s2
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, false);
    }
  }
}

TEST(WireControlCheck, steer) {
  const auto control_period = 0.01;
  const auto steer_delay = 0.5;
  auto check_ptr = std::make_unique<qcraft::control::WireControlChecker>(
      control_period, steer_delay);

  double acceleration_fb = 0.0;
  double acceleration_cmd = 0.0;
  double kappa_fb = 0.0;
  double kappa_cmd = 0.0;
  double speed = 2.1;
  qcraft::Chassis::GearPosition gear_fb = qcraft::Chassis::GEAR_DRIVE;
  qcraft::AutonomyStateProto_State autonomy_state =
      qcraft::AutonomyStateProto::AUTO_DRIVE;

  bool status = false;
  for (int i = 0; i < 200; ++i) {
    if (i < 1) {
      kappa_fb = 0.0;
      kappa_cmd = 0.06;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, false);
    } else if (i < 50) {
      kappa_fb = 0.0;
      kappa_cmd = 0.06;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, true);
    } else if (i < 100) {
      kappa_fb = 0.0;
      kappa_cmd = 0.06;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, true);
    } else if (i < 150) {
      kappa_fb = 0.0;
      kappa_cmd = 0.06;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, true);
    } else {
      kappa_fb = 0.0;
      kappa_cmd = 0.06;
      status = check_ptr->CheckProc(autonomy_state, acceleration_cmd,
                                    acceleration_fb, kappa_cmd, kappa_fb, speed,
                                    gear_fb);
      EXPECT_EQ(status, true);
    }
  }
}
