#include "onboard/control/openloop_control/openloop_control.h"

#include "gtest/gtest.h"
#include "onboard/control/control_defs.h"

namespace qcraft {
namespace control {

TEST(OpenloopControl, Init) {
  const std::string &test_config_path =
      "onboard/control/openloop_control/testdata/calibration.pb.txt";

  auto test_control = std::make_shared<OpenloopControl>();
  auto init = test_control->Init(test_config_path);

  EXPECT_EQ(init, absl::OkStatus());

  ControlCommand cmd;
  ControllerDebugProto controller_debug_proto;
  double speed = 10.0;
  double steer_percentage = 0.0;
  double kappa = 0.0;
  const double steer_ratio = 16.0;
  const double wheel_base = 3.0;
  const double max_steer = 8.2;

  // Define Test type.
  constexpr int testtype = OpenloopCmd::THROTTLEBRAKE;
  constexpr double kEpsilon = 1e-5;
  // TEST
  if (testtype == OpenloopCmd::THROTTLEBRAKE ||
      testtype == OpenloopCmd::ACCELERATION) {
    for (int i = 0; i < 500; ++i) {
      auto result = test_control->Process(speed, steer_ratio, steer_percentage,
                                          kappa, wheel_base, max_steer, &cmd,
                                          &controller_debug_proto);
      EXPECT_EQ(result, absl::OkStatus());

      EXPECT_NEAR(cmd.throttle(), 10.0, kEpsilon);
      EXPECT_NEAR(cmd.brake(), 0.0, kEpsilon);
    }
    for (int i = 500; i < 1000; ++i) {
      auto result = test_control->Process(speed, steer_ratio, steer_percentage,
                                          kappa, wheel_base, max_steer, &cmd,
                                          &controller_debug_proto);
      EXPECT_EQ(result, absl::OkStatus());

      EXPECT_NEAR(cmd.throttle(), 20.0, kEpsilon);
      EXPECT_NEAR(cmd.brake(), 0.0, kEpsilon);
    }
    for (int i = 1000; i < 1500; ++i) {
      auto result = test_control->Process(speed, steer_ratio, steer_percentage,
                                          kappa, wheel_base, max_steer, &cmd,
                                          &controller_debug_proto);
      EXPECT_EQ(result, absl::OkStatus());

      EXPECT_NEAR(cmd.throttle(), 0.0, kEpsilon);
      EXPECT_NEAR(cmd.brake(), 10.0, kEpsilon);
    }
  }

  if (testtype == OpenloopCmd::STEER) {
    speed = 5.0;
    constexpr double kDurationTime = 1.0;
    const int duration_num = kDurationTime / kControlInterval;
    for (int i = 0; i < duration_num; ++i) {
      auto result = test_control->Process(speed, steer_ratio, steer_percentage,
                                          kappa, wheel_base, max_steer, &cmd,
                                          &controller_debug_proto);
      EXPECT_EQ(result, absl::OkStatus());
      EXPECT_NEAR(cmd.steering_target(), 0.0, kEpsilon);
    }
    for (int i = duration_num; i < duration_num + 10; ++i) {
      auto result = test_control->Process(speed, steer_ratio, steer_percentage,
                                          kappa, wheel_base, max_steer, &cmd,
                                          &controller_debug_proto);
      EXPECT_EQ(result, absl::OkStatus());

      constexpr double sample_frequency =
          0.01 / 10.0 * 2 * M_PI;  // steer angle T = 10s;
      double state_steer_percentage =
          5.0 * sin((i - duration_num + 1) * sample_frequency);

      EXPECT_NEAR(cmd.steering_target(), state_steer_percentage, kEpsilon);
    }
  }
}
}  // namespace control
}  // namespace qcraft
