#include "onboard/control/control_validation.h"

#include <array>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/steering_protection.h"
#include "onboard/params/param_manager.h"

namespace qcraft {
namespace control {
namespace {

struct TestCase {
  TrajectoryInterface trajectory_interface;
  std::optional<SteeringProtection> steering_protection;
  VehicleDriveParamsProto vehicle_drive_param;
  VehicleGeometryParamsProto vehicle_geometry_params;
  ControllerConf controller_conf;
  ControlCommand control_cmd;
  ControllerDebugProto controller_debug_proto;

  TestCase() : control_cmd(MakeDefaultControlCmd()) {
    auto param_manager = CreateParamManagerFromCarId("Q0001");
    RunParamsProtoV2 run_params;
    param_manager->GetRunParams(&run_params);
    vehicle_geometry_params =
        run_params.vehicle_params().vehicle_geometry_params();
    vehicle_drive_param = run_params.vehicle_params().vehicle_drive_params();
    controller_conf = run_params.vehicle_params().controller_conf();
    steering_protection.emplace(&vehicle_geometry_params, &vehicle_drive_param,
                                &controller_conf);
  }

  ControlCommand MakeDefaultControlCmd() {
    ControlCommand control_cmd;
    control_cmd.set_acceleration(0.0);
    control_cmd.set_acceleration_offset(0.0);
    control_cmd.set_acceleration_calibration(0.0);
    control_cmd.set_throttle(0.0);
    control_cmd.set_brake(0.0);
    control_cmd.set_curvature(0.0);
    control_cmd.set_steering_target(0.0);

    return control_cmd;
  }
};

void UpdateSteeringProtection(double av_speed, ControlCommand* control_cmd,
                              SteeringProtection* steering_protection) {
  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
  SteeringProtectionResult result =
      steering_protection->CalcKappaAndKappaRateLimit(
          autonomy_state, av_speed, /*front_wheel_angle*/ 0.0,
          /*previous_kappa_cmd*/ 0.0, *control_cmd);
  control_cmd->mutable_debug()
      ->mutable_simple_mpc_debug()
      ->mutable_steering_protection_result()
      ->CopyFrom(result);
}

void UpdateState(double av_speed, TestCase* test_case) {
  UpdateSteeringProtection(av_speed, &test_case->control_cmd,
                           &test_case->steering_protection.value());
}

[[maybe_unused]] TrajectoryProto MakeTrajectoryProto(
    const std::vector<Vec2d>& traj_point) {
  TrajectoryProto trajectory_proto;
  for (const auto& p : traj_point) {
    ApolloTrajectoryPointProto point;
    point.mutable_path_point()->set_x(p.x());
    point.mutable_path_point()->set_y(p.y());

    *trajectory_proto.add_trajectory_point() = point;
  }

  return trajectory_proto;
}

bool ValidateTestCase(TestCase test_case) {
  return ValidateControlOutput(
      test_case.trajectory_interface, test_case.vehicle_drive_param,
      test_case.vehicle_geometry_params, test_case.controller_conf,
      test_case.control_cmd, &test_case.controller_debug_proto);
}

TEST(ValidateControlOutputTest, LongitudinalOutputValidation) {
  // case 0: default value.
  TestCase test_case;
  UpdateState(0.0, &test_case);

  EXPECT_TRUE(ValidateTestCase(test_case));

  // case 1: abnormal mpc acceleration;
  std::array<double, 6> mpc_accel = {-10.0, -2.0, 2.0, 6.0, 10.0, 0.0};
  std::array<bool, 6> res_1 = {false, true, true, false, false, true};

  for (int i = 0; i < mpc_accel.size(); ++i) {
    test_case.control_cmd.set_acceleration(mpc_accel[i]);
    EXPECT_EQ(ValidateTestCase(test_case), res_1[i])
        << "mpc_accel: " << mpc_accel[i];
  }

  // case 2: acceleration offset tests;
  std::array<double, 7> accel_offset = {-3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 0.0};
  const double accel_offset_threshold =
      kSinSlopeLimit * kGravitationalAcceleration;
  for (int i = 0; i < accel_offset.size(); ++i) {
    const double offset = accel_offset[i];
    test_case.control_cmd.set_acceleration_offset(offset);

    EXPECT_EQ(ValidateTestCase(test_case),
              InRange(offset, -accel_offset_threshold, accel_offset_threshold))
        << "accel_offset: " << offset;
  }

  // case 3: acceleration calibration tests;
  std::array<double, 6> accel_calib = {-10.0, -2.0, 2.0, 8.0, 10.0, 0.0};
  std::array<bool, 6> res_3 = {false, true, true, false, false, true};

  for (int i = 0; i < accel_calib.size(); ++i) {
    test_case.control_cmd.set_acceleration_calibration(accel_calib[i]);
    EXPECT_EQ(ValidateTestCase(test_case), res_3[i])
        << "accel_calib: " << accel_calib[i];
  }

  // case 4: throttle or brake command tests;
  std::array<double, 10> cmd = {-200.0, -150.0, -100.0, -99.0, -30.0,
                                50.0,   100.0,  101.0,  600.0, 0.0};
  for (int i = 0; i < cmd.size(); ++i) {
    test_case.control_cmd.set_throttle(cmd[i] >= 0.0 ? cmd[i] : 0.0);
    test_case.control_cmd.set_brake(cmd[i] < 0.0 ? -cmd[i] : 0.0);
    EXPECT_EQ(ValidateTestCase(test_case),
              InRange(std::fabs(cmd[i]), 0.0, 100.0))
        << "cmd: " << cmd[i];
  }
}

TEST(ValidateControlOutputTest, LateralOutputValidation) {
  // case 0: default value.
  TestCase test_case;
  UpdateState(0.0, &test_case);

  EXPECT_TRUE(ValidateTestCase(test_case));

  // case 1: curvature command tests;
  const double min_turn_radius =
      test_case.vehicle_geometry_params.has_min_turn_radius()
          ? test_case.vehicle_geometry_params.min_turn_radius()
          : 6.0;
  const auto& steering_protection_result = test_case.control_cmd.debug()
                                               .simple_mpc_debug()
                                               .steering_protection_result();
  const double geo_limit =
      std::min(steering_protection_result.kappa_limit_wrt_geometry(),
               1.0 / min_turn_radius);
  const double speed_limit = steering_protection_result.kappa_limit_wrt_speed();
  const double curvature_limit = std::min(geo_limit, speed_limit);
  std::array<double, 8> curvature_test = {-10.0, -4.0, 0.05, 0.03,
                                          8.0,   0.05, 3.0,  0.0};

  for (int i = 0; i < curvature_test.size(); ++i) {
    test_case.control_cmd.set_curvature(curvature_test[i]);
    EXPECT_EQ(ValidateTestCase(test_case),
              InRange(std::fabs(curvature_test[i]), -curvature_limit,
                      curvature_limit))
        << "curvature_test[" << i << "]: " << curvature_test[i]
        << ", curvature_limit: " << curvature_limit;
  }

  // case 2: steering command tests;
  std::array<double, 11> steer_cmd = {-120.0, -100.1, -100.0, -99.99,
                                      -50.0,  0.0,    33.3,   99.9,
                                      100.0,  101.0,  120.0};

  for (int i = 0; i < steer_cmd.size(); ++i) {
    test_case.control_cmd.set_steering_target(steer_cmd[i]);
    EXPECT_EQ(ValidateTestCase(test_case),
              InRange(steer_cmd[i], -100.01, 100.01))
        << "steer_cmd[" << i << "]: " << steer_cmd[i] << " is out of range.";
  }

  // case 3: high speed, steering command tests;
  UpdateState(Kph2Mps(100.0), &test_case);
  std::array<double, 8> steer_cmd_high_speed = {-00.0, -99.9, -60.0, -3.0,
                                                0.0,   2.5,   50.0,  100.0};
  constexpr double kHighSpeedSteerThreshold = 30.0;
  for (int i = 0; i < steer_cmd_high_speed.size(); ++i) {
    test_case.control_cmd.set_steering_target(steer_cmd_high_speed[i]);
    EXPECT_EQ(ValidateTestCase(test_case),
              InRange(steer_cmd_high_speed[i], -kHighSpeedSteerThreshold,
                      kHighSpeedSteerThreshold))
        << "steer_cmd_high_speed[" << i << "]: " << steer_cmd_high_speed[i]
        << " is out of range.";
  }
}

// TODO(zhichao): add tests to validate mpc prediction deviation.

}  // namespace
}  // namespace control
}  // namespace qcraft
