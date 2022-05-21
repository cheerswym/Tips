#include "onboard/control/steering_protection.h"

#include "gtest/gtest.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 0.0001;

ControlCommand MakeControlCommand(bool is_steering_back_to_center) {
  ControlCommand control_cmd;
  control_cmd.mutable_debug()
      ->mutable_simple_mpc_debug()
      ->set_is_steering_back_to_center(is_steering_back_to_center);
  return control_cmd;
}

TEST(SteeringProtection, CalcKappaAndKappaRateLimitWhenAvControllableTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto &vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto &control_conf = run_params.vehicle_params().controller_conf();

  constexpr int kMpcHorizon = 10;
  double previous_kappa_cmd = 0.1;
  double av_speed_list[] = {-5.0, -1.0, 0.0, 0.05, 1.0, 5.0};
  double front_wheel_angle_list[] = {-0.1, -0.01, 0.0, 0.01, 0.1};
  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);

  for (auto &av_speed : av_speed_list) {
    for (auto &front_wheel_angle : front_wheel_angle_list) {
      SteeringProtection steering_protection(
          &vehicle_geo_params, &vehicle_drive_params, &control_conf);

      const bool is_steering_back_center = false;
      ControlCommand control_cmd = MakeControlCommand(is_steering_back_center);
      const double lateral_jerk_limit = is_steering_back_center
                                            ? kLateralkJerkLimitLow
                                            : kLateralkJerkLimitHigh;
      // Kappa rate ground truth.
      constexpr double kSpeedLowerLimit = 0.1;  // m/s.
      const double v = std::max(std::fabs(av_speed), kSpeedLowerLimit);

      const double kappa_rate_limit_static =
          SteeringSpeed2KappaRateWithFrontWheelAngle(
              kSteeringSpeedLimit, front_wheel_angle,
              vehicle_geo_params.wheel_base(),
              vehicle_drive_params.steer_ratio());

      const double kappa_rate_limit_wrt_lateral_jerk =
          lateral_jerk_limit / Sqr(v);
      const double kappa_rate_limit_final =
          std::min(kappa_rate_limit_static, kappa_rate_limit_wrt_lateral_jerk);

      // Kappa limit ground truth.
      const double kappa_limit_wrt_geometry = SteerAngle2Kappa(
          vehicle_drive_params.max_steer_angle(),
          vehicle_geo_params.wheel_base(), vehicle_drive_params.steer_ratio());

      const double kappa_limit_wrt_speed =
          control_conf.max_lateral_acceleration() / Sqr(v);

      const double kappa_limit_wrt_geometry_and_speed =
          std::min(kappa_limit_wrt_geometry, kappa_limit_wrt_speed);

      SteeringProtectionResult result =
          steering_protection.CalcKappaAndKappaRateLimit(
              autonomy_state, av_speed, front_wheel_angle, previous_kappa_cmd,
              control_cmd);

      EXPECT_NEAR(result.kappa_rate_upper(), kappa_rate_limit_final, kEpsilon)
          << "kappa rate limit final: " << kappa_rate_limit_final;
      EXPECT_NEAR(result.kappa_rate_lower(), -kappa_rate_limit_final, kEpsilon);

      EXPECT_EQ(result.kappa_output_upper_size(), kMpcHorizon)
          << "kappa output upper size: " << result.kappa_output_upper_size()
          << " | expect result: " << kMpcHorizon;
      EXPECT_EQ(result.kappa_output_lower_size(), kMpcHorizon);

      double kappa_output_uppper =
          previous_kappa_cmd + kappa_rate_limit_final * kControlInterval;
      double kappa_output_lower =
          previous_kappa_cmd - kappa_rate_limit_final * kControlInterval;
      for (int i = 0; i < kMpcHorizon; ++i) {
        EXPECT_NEAR(result.kappa_output_upper(i), kappa_output_uppper, kEpsilon)
            << "index: " << i;
        EXPECT_NEAR(result.kappa_output_lower(i), kappa_output_lower, kEpsilon);
        kappa_output_uppper += kappa_rate_limit_final *
                               control_conf.ts_pkmpc_controller_conf().ts();
        kappa_output_uppper =
            std::min(kappa_output_uppper, kappa_limit_wrt_geometry_and_speed);
        kappa_output_lower -= kappa_rate_limit_final *
                              control_conf.ts_pkmpc_controller_conf().ts();
        kappa_output_lower =
            std::max(kappa_output_lower, -kappa_limit_wrt_geometry_and_speed);
      }
    }
  }
}

TEST(SteeringProtection,
     CalcKappaAndKappaRateLimitTestWhenAvUncontrollableTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto &vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto &control_conf = run_params.vehicle_params().controller_conf();

  constexpr int kMpcHorizon = 10;
  double previous_kappa_cmd = 0.1;
  double front_wheel_angle_list[] = {-0.1, -0.01, 0.0, 0.01, 0.1};
  const double av_speed = 10.0;

  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::READY_TO_AUTO_DRIVE);

  for (auto &front_wheel_angle : front_wheel_angle_list) {
    SteeringProtection steering_protection(
        &vehicle_geo_params, &vehicle_drive_params, &control_conf);

    const bool is_steering_back_center = false;
    ControlCommand control_cmd = MakeControlCommand(is_steering_back_center);

    // Kappa rate ground truth.
    const double kappa_rate_limit_static =
        SteeringSpeed2KappaRateWithFrontWheelAngle(
            kSteeringSpeedLimit, front_wheel_angle,
            vehicle_geo_params.wheel_base(),
            vehicle_drive_params.steer_ratio());

    // Kappa limit ground truth.
    const double kappa_limit_wrt_geometry = SteerAngle2Kappa(
        vehicle_drive_params.max_steer_angle(), vehicle_geo_params.wheel_base(),
        vehicle_drive_params.steer_ratio());

    SteeringProtectionResult result =
        steering_protection.CalcKappaAndKappaRateLimit(
            autonomy_state, av_speed, front_wheel_angle, previous_kappa_cmd,
            control_cmd);

    EXPECT_NEAR(result.kappa_rate_upper(), kappa_rate_limit_static, kEpsilon);
    EXPECT_NEAR(result.kappa_rate_lower(), -kappa_rate_limit_static, kEpsilon);

    EXPECT_EQ(result.kappa_output_upper_size(), kMpcHorizon);
    EXPECT_EQ(result.kappa_output_lower_size(), kMpcHorizon);

    for (int i = 0; i < kMpcHorizon; ++i) {
      EXPECT_NEAR(result.kappa_output_upper(i), kappa_limit_wrt_geometry,
                  kEpsilon);
      EXPECT_NEAR(result.kappa_output_lower(i), -kappa_limit_wrt_geometry,
                  kEpsilon);
    }
  }
}

// Test whether unreasonable previous_kappa_cmd will lead to crash.
TEST(SteeringProtection, CalcKappaAndKappaRateLimitCrashTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto &vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto &control_conf = run_params.vehicle_params().controller_conf();

  const bool is_steering_back_center = false;
  ControlCommand control_cmd = MakeControlCommand(is_steering_back_center);

  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);

  const double av_speed = 5.0;                 // m/s.
  const double front_wheel_angle = 15 / M_PI;  // rad.
  const double previous_kappa_cmd = -0.5;

  SteeringProtection steering_protection(&vehicle_geo_params,
                                         &vehicle_drive_params, &control_conf);
  SteeringProtectionResult result =
      steering_protection.CalcKappaAndKappaRateLimit(
          autonomy_state, av_speed, front_wheel_angle, previous_kappa_cmd,
          control_cmd);
}

TEST(SteeringProtection, IsProtectiveKickoutTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto &vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto &vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  auto control_conf = run_params.vehicle_params().controller_conf();
  control_conf.clear_active_controllers();
  control_conf.add_active_controllers(ControllerConf::TS_PKMPC_CONTROLLER);

  SteeringProtection steering_protection(&vehicle_geo_params,
                                         &vehicle_drive_params, &control_conf);

  ControlCommand control_cmd;
  ControllerDebugProto controller_debug_proto;

  SteeringProtectionResult result;
  constexpr double kKappaRateUpper = 0.1;  // rad/s.
  result.set_kappa_rate_upper(kKappaRateUpper);

  ControlHistoryStateManager control_history_state_mgr;
  const int cb_capacity =
      control_history_state_mgr.GetControlStateCache().capacity();
  const double av_speed = 5;

  // Case 1: Cache data is not full.
  for (int i = 0; i < cb_capacity - 2; ++i) {
    const double kappa_cmd = i * kKappaRateUpper;
    const bool is_under_control = true;
    const double steering_pct_chassis = 0.0;
    const double acc_target = 0.0;
    control_history_state_mgr.UpdateHistoryData(
        is_under_control, kappa_cmd, steering_pct_chassis, acc_target);
  }
  EXPECT_FALSE(control_history_state_mgr.GetControlStateCache().full());
  EXPECT_FALSE(steering_protection.IsProtectiveKickout(
      av_speed, control_history_state_mgr.GetControlStateCache(),
      controller_debug_proto, &result));

  // Case 2: the current autonomy state is not under control.
  for (int i = 0; i < cb_capacity + 1; ++i) {
    const double kappa_cmd = i * kKappaRateUpper;
    const bool is_under_control = true;
    const double steering_pct_chassis = 0.0;
    const double acc_target = 0.0;
    control_history_state_mgr.UpdateHistoryData(
        is_under_control, kappa_cmd, steering_pct_chassis, acc_target);
  }
  control_history_state_mgr.UpdateHistoryData(false, 0.0, 0.0, 0.0);

  EXPECT_FALSE(steering_protection.IsProtectiveKickout(
      av_speed, control_history_state_mgr.GetControlStateCache(),
      controller_debug_proto, &result));

  // Case 3: Consider steering speed only in past time without mpc predict
  // horizon.
  const double relax_factor_list[] = {-1.1,
                                      -kRelaxFactorThreshold - 0.01,
                                      -kRelaxFactorThreshold + 0.01,
                                      -0.5,
                                      0.0,
                                      0.5,
                                      kRelaxFactorThreshold - 0.01,
                                      kRelaxFactorThreshold + 0.01,
                                      1.1};
  const double av_speed_list[] = {-2.0, -1.0, 0.0, 1.0, 2.0};

  for (const auto &av_speed : av_speed_list) {
    for (const auto &relax_factor : relax_factor_list) {
      for (int i = 0; i < cb_capacity + 1; ++i) {
        const double kappa_cmd =
            i * relax_factor * kKappaRateUpper * kControlInterval;
        const bool is_under_control = true;
        const double steering_pct_chassis = 0.0;
        const double acc_target = 0.0;
        control_history_state_mgr.UpdateHistoryData(
            is_under_control, kappa_cmd, steering_pct_chassis, acc_target);
      }
      EXPECT_EQ(steering_protection.IsProtectiveKickout(
                    av_speed, control_history_state_mgr.GetControlStateCache(),
                    controller_debug_proto, &result),
                std::fabs(relax_factor) >= kRelaxFactorThreshold)
          << "relax_factor: " << relax_factor << " | av_speed: " << av_speed
          << " | kappa_rate_actual_mean: " << result.kappa_rate_actual_mean()
          << " | kappa rate upper: " << result.kappa_rate_upper();
      EXPECT_NEAR(result.kappa_rate_actual_mean(),
                  kKappaRateUpper * relax_factor, 1e-5);
    }
  }

  // Case 4: Consider the steering speed in past time and ts pk mpc predict
  // horizon.
  const double relax_factor_list_1[] = {
      -kRelaxFactorThreshold - 0.01, -kRelaxFactorThreshold + 0.01,
      kRelaxFactorThreshold - 0.01, kRelaxFactorThreshold + 0.01};
  for (const auto &actual_relax_factor : relax_factor_list_1) {
    for (const auto &predict_relax_factor : relax_factor_list_1) {
      for (int i = 0; i < cb_capacity + 1; ++i) {
        const double kappa_cmd =
            i * actual_relax_factor * kKappaRateUpper * kControlInterval;
        const bool is_under_control = true;
        const double steering_pct_chassis = 0.0;
        const double acc_target = 0.0;
        control_history_state_mgr.UpdateHistoryData(
            is_under_control, kappa_cmd, steering_pct_chassis, acc_target);
      }
      constexpr int kMpcHorizon = 10;
      auto *mpc_debug = controller_debug_proto.mutable_mpc_debug_proto();
      mpc_debug->Clear();
      for (int i = 0; i < kMpcHorizon; ++i) {
        mpc_debug->add_s_control_mpc_result(
            i * predict_relax_factor * kKappaRateUpper *
            control_conf.ts_pkmpc_controller_conf().ts());
      }

      const bool is_protective_kickout_gt =
          std::fabs(actual_relax_factor) >= kRelaxFactorThreshold &&
          std::fabs(predict_relax_factor) >= kRelaxFactorThreshold &&
          actual_relax_factor * predict_relax_factor > 0.0;
      EXPECT_EQ(steering_protection.IsProtectiveKickout(
                    av_speed, control_history_state_mgr.GetControlStateCache(),
                    controller_debug_proto, &result),
                is_protective_kickout_gt)
          << "actual_relax_factor: " << actual_relax_factor
          << " | predict_relax_factor: " << predict_relax_factor
          << " | kappa_rate_actual_mean: " << result.kappa_rate_actual_mean()
          << " | kappa rate future mean: " << result.kappa_rate_predict_mean();

      EXPECT_NEAR(result.kappa_rate_actual_mean(),
                  kKappaRateUpper * actual_relax_factor, 1e-5);
      EXPECT_NEAR(result.kappa_rate_predict_mean(),
                  kKappaRateUpper * predict_relax_factor, 1e-5);
    }
  }

  // Case 5: Consider the steering speed in past time and tob ts pk mpc predict
  // horizon.
  control_conf.clear_active_controllers();
  control_conf.add_active_controllers(ControllerConf::TOB_TSPKMPC_CONTROLLER);

  SteeringProtection steering_protection_1(
      &vehicle_geo_params, &vehicle_drive_params, &control_conf);
  for (const auto &actual_relax_factor : relax_factor_list_1) {
    for (const auto &predict_relax_factor : relax_factor_list_1) {
      for (int i = 0; i < cb_capacity + 1; ++i) {
        const double kappa_cmd =
            i * actual_relax_factor * kKappaRateUpper * kControlInterval;
        const bool is_under_control = true;
        const double steering_pct_chassis = 0.0;
        const double acc_target = 0.0;
        control_history_state_mgr.UpdateHistoryData(
            is_under_control, kappa_cmd, steering_pct_chassis, acc_target);
      }
      constexpr int kMpcHorizon = 10;
      auto *mpc_debug = controller_debug_proto.mutable_mpc_debug_proto();
      mpc_debug->Clear();
      for (int i = 0; i < kMpcHorizon; ++i) {
        mpc_debug->add_s_control_mpc_result(predict_relax_factor *
                                            kKappaRateUpper);
      }

      const bool is_protective_kickout_gt =
          std::fabs(actual_relax_factor) >= kRelaxFactorThreshold &&
          std::fabs(predict_relax_factor) >= kRelaxFactorThreshold &&
          actual_relax_factor * predict_relax_factor > 0.0;
      EXPECT_EQ(steering_protection_1.IsProtectiveKickout(
                    av_speed, control_history_state_mgr.GetControlStateCache(),
                    controller_debug_proto, &result),
                is_protective_kickout_gt)
          << "actual_relax_factor: " << actual_relax_factor
          << " | predict_relax_factor: " << predict_relax_factor
          << " | kappa_rate_actual_mean: " << result.kappa_rate_actual_mean()
          << " | kappa rate future mean: " << result.kappa_rate_predict_mean();

      EXPECT_NEAR(result.kappa_rate_actual_mean(),
                  kKappaRateUpper * actual_relax_factor, 1e-5);
      EXPECT_NEAR(result.kappa_rate_predict_mean(),
                  kKappaRateUpper * predict_relax_factor, 1e-5);
    }
  }
}

}  // namespace
}  // namespace qcraft::control
