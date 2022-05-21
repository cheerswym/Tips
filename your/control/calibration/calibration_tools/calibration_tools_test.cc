#include "onboard/control/calibration/calibration_tools/calibration_tools.h"

#include <cmath>
#include <string>

#include "gtest/gtest.h"

const char dirname[] = "onboard/control/calibration/calibration_tools/testdata";
const char filename[] = "Q1002";

TEST(ControlCalibration, IDLE) {
  const char method[] = "IDLE";
  auto calibration_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationTools>(
          dirname, filename, method, 0.0, 0.0, true);
  double speed = 0.0;
  double acceleration = 0.0;
  qcraft::vis::vantage::ChartDataProto chart;
  for (int i = 0; i < 200; i++) {
    speed = 0.01 * i;
    acceleration =
        -0.12778741 * speed * speed - 0.06507266 * speed + 0.32284199;
    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, 0.0, 0.0,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
}

TEST(ControlCalibration, SLIDE) {
  const char method[] = "SLIDE";
  auto calibration_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationTools>(
          dirname, filename, method, 0.0, 0.0, true);
  double speed = 0.0;
  double acceleration = 0.0;
  qcraft::vis::vantage::ChartDataProto chart;
  for (int i = 0; i < 1500; i++) {
    acceleration = 1.0;
    speed += 0.01 * acceleration;
    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, 0.0, 0.0,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
  for (int i = 2000; i >= 150; i--) {
    speed = 0.01 * i;
    acceleration = 0.0010 * speed * speed - 0.0670 * speed + 0.0593;
    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, 0.0, 0.0,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
}

TEST(ControlCalibration, FORCE) {
  const char method[] = "FORCE";
  auto calibration_ptr =
      std::make_unique<qcraft::control::calibration::CalibrationTools>(
          dirname, filename, method, 0.0, 0.0, true);
  double speed = 0.0;
  double acceleration = 0.0;
  double percentage = 0.0;
  double throttle = 0.0;
  double brake = 0.0;
  qcraft::vis::vantage::ChartDataProto chart;
  for (int i = 0; i < 1500; i++) {
    acceleration = std::sin(0.01 * i);
    speed += 0.01 * acceleration;
    percentage = 100.0 * std::sin(0.01 * i);
    if (percentage >= 0.0) {
      throttle = percentage;
    } else {
      brake = -percentage;
    }

    const auto status =
        calibration_ptr->Process(speed, acceleration, 0.0, throttle, brake,
                                 qcraft::Chassis::GEAR_DRIVE, &chart);
    EXPECT_EQ(status, true);
  }
}
