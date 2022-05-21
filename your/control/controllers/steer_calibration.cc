#include "onboard/control/controllers/steer_calibration.h"

#include "onboard/lite/logging.h"

namespace qcraft {
namespace control {
namespace {
constexpr double kMaxDynamicGain = 2.5;
constexpr double kMinDynamicGain = 0.99;
double DynamicGain(double speed, double sliding_factor) {
  double dynamic_gain = (1 - sliding_factor * speed * speed);
  QCHECK(dynamic_gain > kMinDynamicGain && dynamic_gain < kMaxDynamicGain)
      << "[control] Dynamic_gain is over bound. dynamic_gain: " << dynamic_gain;
  return dynamic_gain;
}
}  // namespace

double SteerCalibration::DeadzoneGain(double steer_kappa,
                                      double wheelbase) const {
  const double s_steer_rad_abs =
      std::abs(Kappa2FrontWheelAngle(steer_kappa, wheelbase));
  QCHECK_GT(steer_turn_th_, steer_straight_th_)
      << "[control] steer_straight_th <= steer_turn_th.";
  double deadzone_gain = 1.0;

  if (s_steer_rad_abs < steer_straight_th_) {
    deadzone_gain = steer_straight_gain_;
  } else if (s_steer_rad_abs > steer_turn_th_) {
    deadzone_gain = steer_turn_gain_;
  } else {
    // Documentation: https://qcraft.feishu.cn/docs/doccnYpWacAMAiqBEFaZISnXsAc
    const PiecewiseLinearFunction<double> deadzone_plf(
        {steer_straight_th_, steer_turn_th_},
        {steer_straight_gain_ * steer_straight_th_,
         steer_turn_gain_ * steer_turn_th_});
    deadzone_gain = deadzone_plf.Evaluate(s_steer_rad_abs) / s_steer_rad_abs;
  }

  return deadzone_gain;
}

double SteerCalibration::AntiGapCompensate(double steer_kappa,
                                           double steer_kappa_past) {
  const double KMaxDiffKappaCompensate = 0.00002;
  const double KDiffKappathreshold = 0.000003;
  if ((anti_gap_sign_ > 0.0) &&
      (steer_kappa - steer_kappa_past < -KDiffKappathreshold)) {
    anti_gap_sign_ = -1.0;
  } else if ((anti_gap_sign_ < 0.0) &&
             (steer_kappa - steer_kappa_past > KDiffKappathreshold)) {
    anti_gap_sign_ = 1.0;
  }
  double steer_gap_kappa_compensate = steer_gap_kappa_compensate_last_ +
                                      KMaxDiffKappaCompensate * anti_gap_sign_;
  steer_gap_kappa_compensate = std::clamp(steer_gap_kappa_compensate,
                                          -steer_gap_kappa_, steer_gap_kappa_);

  steer_gap_kappa_compensate_last_ = steer_gap_kappa_compensate;
  return steer_gap_kappa_compensate;
}

double SteerCalibration::SteerCalibrationMain(
    double steer_kappa, double steer_kappa_past, double speed,
    double wheel_base, double steer_ratio, double max_steer_angle,
    SteerCalibrationDebugProto* debug) {
  double steer_gain = 1.0;
  double speed_gain = 1.0;
  double deadzone_gain = 1.0;
  double dynamic_gain = 1.0;

  if (enable_adapt_deadzone_) {
    deadzone_gain = DeadzoneGain(steer_kappa, wheelbase_);
    steer_gain *= deadzone_gain;
  }
  if (enable_dynamic_model_compensation_) {
    dynamic_gain = DynamicGain(speed, sliding_factor_);
    steer_gain *= dynamic_gain;
  } else if (has_steering_gain_wrt_speed_) {
    speed_gain = steering_gain_wrt_speed_plf_(speed);
    steer_gain *= speed_gain;
  }
  double steer_gap_kappa_compensate =
      AntiGapCompensate(steer_kappa, steer_kappa_past);

  const double output_steer_kappa =
      steer_kappa * steer_gain + steer_gap_kappa_compensate;
  const double output_steer_percentage =
      std::clamp(Kappa2SteerPercentage(output_steer_kappa, wheel_base,
                                       steer_ratio, max_steer_angle),
                 -100.0, 100.0);

  debug->set_speed_gain(speed_gain);
  debug->set_deadzone_gain(deadzone_gain);
  debug->set_dynamic_gain(dynamic_gain);
  debug->set_post_process_gain(steer_gain);
  debug->set_steer_gap_kappa_compensate(steer_gap_kappa_compensate);
  debug->set_input_steer_kappa(steer_kappa);
  debug->set_output_steer_kappa(output_steer_kappa);
  debug->set_output_steer_percentage(output_steer_percentage);

  return output_steer_percentage;
}

SteerCalibration::SteerCalibration(const ControllerConf& control_conf,
                                   double wheel_base) {
  const auto& dynamic_conf = control_conf.veh_dynamic_model_conf();
  enable_dynamic_model_compensation_ =
      dynamic_conf.enable_dynamic_model_compensation();
  mf_ = dynamic_conf.mass_fl() + dynamic_conf.mass_fr();
  mr_ = dynamic_conf.mass_rl() + dynamic_conf.mass_rr();
  cf_ = dynamic_conf.c_fl() + dynamic_conf.c_fr();
  cr_ = dynamic_conf.c_rl() + dynamic_conf.c_rr();
  wheelbase_f = dynamic_conf.wheelbase_f();
  wheelbase_r = dynamic_conf.wheelbase_r();
  wheelbase_ = wheel_base;
  if (enable_dynamic_model_compensation_ &&
      (std::fabs(wheelbase_ * wheelbase_ * cf_ * cr_) > 0.01)) {
    sliding_factor_ = (mf_ + mr_) * (cf_ * wheelbase_f - cr_ * wheelbase_r) /
                      (wheelbase_ * wheelbase_ * cf_ * cr_);
  }

  has_steering_gain_wrt_speed_ = control_conf.has_steering_gain_wrt_speed();
  if (has_steering_gain_wrt_speed_) {
    steering_gain_wrt_speed_plf_ = PiecewiseLinearFunctionFromProto(
        control_conf.steering_gain_wrt_speed());
  }

  const auto& deadzone_conf = control_conf.steer_deadzone_adaptor_conf();
  enable_adapt_deadzone_ = deadzone_conf.enable_adapt_deadzone();
  steer_straight_th_ = deadzone_conf.steer_straight_th();
  steer_turn_th_ = deadzone_conf.steer_turn_th();
  steer_straight_gain_ = deadzone_conf.steer_straight_gain();
  steer_turn_gain_ = deadzone_conf.steer_turn_gain();
  steer_gap_kappa_ = deadzone_conf.steer_gap_kappa();
}

}  // namespace control
}  // namespace qcraft
