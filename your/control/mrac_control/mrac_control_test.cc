#include "onboard/control/mrac_control/mrac_control.h"

#include "gtest/gtest.h"
#include "onboard/control/controllers/controller_util.h"

TEST(MracControl, reset) {
  constexpr double kEpsilon = 1e-5;
  // Init
  qcraft::control::MracConfig mrac_config;
  mrac_config.steer_delay = 0.1;
  mrac_config.mrac_conf.set_matrix_p_1(435.0);
  mrac_config.mrac_conf.set_state_weight(0.3);
  mrac_config.mrac_conf.set_input_weight(1.0);
  mrac_config.mrac_conf.set_cutoff_frequency(20.0);
  mrac_config.mrac_conf.set_damping_ratio(0.707);
  mrac_config.mrac_conf.set_ke_min_weight(0.0);
  mrac_config.mrac_conf.set_ke_max_weight(0.2);
  mrac_config.mrac_conf.set_max_error(0.02);
  mrac_config.mrac_conf.set_max_delta_kx(0.02);
  mrac_config.mrac_conf.set_max_delta_kr(0.02);
  mrac_config.mrac_conf.set_min_kr(0.8);
  mrac_config.mrac_conf.set_max_kr(1.2);
  mrac_config.mrac_conf.set_min_kx(-0.2);
  mrac_config.mrac_conf.set_max_kx(0.2);
  mrac_config.mrac_conf.set_speed_limit(0.2);
  mrac_config.mrac_conf.set_kappa_error_window(5);
  mrac_config.mrac_conf.set_kappa_threshold(0.02);

  auto mrac_control_ptr =
      std::make_unique<qcraft::control::MracControl>(mrac_config);
  const auto init = mrac_control_ptr->Init();

  EXPECT_EQ(init, absl::OkStatus());

  qcraft::MracDebugProto debug;
  // speed < limit
  const double kappa_target = 0.04;
  const double av_kappa = 0.04;
  const double speed = 0.1;
  const bool is_automode = true;

  const qcraft::control::KappaConstraint kappa_constraint = {
      .kappa_upper = 12.5,
      .kappa_lower = -12.5,
      .kappa_rate_upper = 0.15,
      .kappa_rate_lower = -0.15,
  };

  const double kappa_cmd = mrac_control_ptr->MracComputer(
      is_automode, kappa_target, av_kappa, speed, kappa_constraint, &debug);

  EXPECT_EQ(debug.mrac_state(), true);
  EXPECT_NEAR(kappa_cmd, kappa_target, kEpsilon);
  EXPECT_NEAR(debug.av_kappa(), av_kappa, kEpsilon);
  EXPECT_NEAR(debug.kappa_input(), kappa_target, kEpsilon);
}
