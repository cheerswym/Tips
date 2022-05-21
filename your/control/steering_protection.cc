#include "onboard/control/steering_protection.h"

#include <algorithm>
#include <vector>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "offboard/vis/vantage/charts/chart_util.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/math/util.h"

namespace qcraft::control {

void WrapSteerConstraintChartData(
    double prev_kappa_cmd, const ControllerConf &controller_conf,
    const ControlCommand &control_command,
    const ControllerDebugProto &controller_debug_proto,
    vis::vantage::ChartsDataProto *chart_data) {
  SCOPED_QTRACE("WrapSteerConstraintChartData");
  if (controller_debug_proto.mpc_debug_proto().s_control_mpc_result_size() <
      kSControlHorizon) {
    return;
  }

  const double time_step = controller_conf.ts_pkmpc_controller_conf().ts();
  const bool is_mpc_cmd_in_kappa = controller_conf.active_controllers()[0] ==
                                   ControllerConf::TS_PKMPC_CONTROLLER;

  constexpr int kChartMemberSize = 9;
  std::vector<double> x_t;
  std::vector<std::string> names = {
      "kappa_geo_ul",   "kappa_geo_ll",  "kappa_speed_ul",
      "kappa_speed_ll", "psi_static_ul", "psi_static_ll",
      "psi_jerk_ul",    "psi_jerk_ll",   "mpc_kappa"};
  // TODO(zhichao, zhenxing): add mrac kappa result and planner reference kappa
  // series into this debug chart.
  std::vector<std::vector<double>> values(kChartMemberSize);

  const auto &steering_protection =
      control_command.debug().simple_mpc_debug().steering_protection_result();
  const double kappa_geo_limit = steering_protection.kappa_limit_wrt_geometry();
  const double kappa_speed_limit = steering_protection.kappa_limit_wrt_speed();
  // psi = d(kappa)/dt;
  const double psi_limit_static = steering_protection.kappa_rate_limit_static();
  const double psi_limit_wrt_lateral_jerk =
      steering_protection.kappa_rate_limit_wrt_lateral_jerk();

  double kappa_cmd = prev_kappa_cmd;

  for (int i = -1; i < kSControlHorizon; ++i) {
    const double t = i * time_step;
    x_t.push_back(t);
    // kappa limit;
    values[0].push_back(kappa_geo_limit);
    values[1].push_back(-kappa_geo_limit);
    values[2].push_back(kappa_speed_limit);
    values[3].push_back(-kappa_speed_limit);
    // psi limit;
    const double delta_kappa_wrt_psi_static =
        (t + time_step) * psi_limit_static;
    const double delta_kappa_wrt_psi_jerk =
        (t + time_step) * psi_limit_wrt_lateral_jerk;
    values[4].push_back(prev_kappa_cmd + delta_kappa_wrt_psi_static);
    values[5].push_back(prev_kappa_cmd - delta_kappa_wrt_psi_static);
    values[6].push_back(prev_kappa_cmd + delta_kappa_wrt_psi_jerk);
    values[7].push_back(prev_kappa_cmd - delta_kappa_wrt_psi_jerk);
    // mpc kappa.
    if (i == -1) {
      values[8].push_back(prev_kappa_cmd);
      continue;
    }
    const double mpc_cmd =
        controller_debug_proto.mpc_debug_proto().s_control_mpc_result()[i];
    kappa_cmd += mpc_cmd * time_step;
    values[8].push_back(is_mpc_cmd_in_kappa ? mpc_cmd : kappa_cmd);
  }

  vis::vantage::GenerateChartFromData("control/steer_constraint", x_t, values,
                                      "t", names, chart_data->add_charts());
}

SteeringProtectionResult SteeringProtection::CalcKappaAndKappaRateLimit(
    const AutonomyStateProto &autonomy_state, double av_speed,
    double front_wheel_angle, double previous_kappa_cmd,
    const ControlCommand &control_command) const {
  SCOPED_QTRACE("SteeringProtection::CalcKappaAndKappaRateLimit");
  SteeringProtectionResult steering_protection_result;
  /*
   * Step1: Calculate kappa limit.
   */
  const double kappa_limit_wrt_geometry =
      SteerAngle2Kappa(vehicle_drive_params_->max_steer_angle(),
                       vehicle_geometry_params_->wheel_base(),
                       vehicle_drive_params_->steer_ratio());

  constexpr double kSpeedLowerLimit = 0.1;  // m/s.
  const double v = std::max(std::fabs(av_speed), kSpeedLowerLimit);
  const double kappa_limit_wrt_speed =
      control_conf_->max_lateral_acceleration() / Sqr(v);

  const double kappa_limit_wrt_geometry_and_speed =
      std::min(kappa_limit_wrt_geometry, kappa_limit_wrt_speed);

  /*
   * Step2: Calculate kappa rate limit.
   */
  const double kappa_rate_limit_static =
      SteeringSpeed2KappaRateWithFrontWheelAngle(
          kSteeringSpeedLimit, front_wheel_angle,
          vehicle_geometry_params_->wheel_base(),
          vehicle_drive_params_->steer_ratio());

  const bool is_steering_back_to_center =
      control_command.debug().simple_mpc_debug().is_steering_back_to_center();

  // Calculate kappa_rate_limit based on lateral jerk limitation.
  const double lateral_jerk_limitation = is_steering_back_to_center
                                             ? kLateralkJerkLimitLow
                                             : kLateralkJerkLimitHigh;
  // Lateral jerk ~= v^2 * psi;
  const double kappa_rate_limit_wrt_lateral_jerk =
      lateral_jerk_limitation / Sqr(v);
  const double kappa_rate_limit_final =
      std::min(kappa_rate_limit_static, kappa_rate_limit_wrt_lateral_jerk);
  QCHECK_GT(kappa_rate_limit_final, 0.0)
      << "kappa_rate_limit_final = " << kappa_rate_limit_final;

  /*
   * Step3: Record result.
   */
  steering_protection_result.set_kappa_limit_wrt_geometry(
      kappa_limit_wrt_geometry);
  steering_protection_result.set_kappa_limit_wrt_speed(kappa_limit_wrt_speed);
  steering_protection_result.set_kappa_rate_limit_static(
      kappa_rate_limit_static);
  steering_protection_result.set_kappa_rate_limit_wrt_lateral_jerk(
      kappa_rate_limit_wrt_lateral_jerk);

  /*
   * Step4: Calculate kappa output upper and lower sequence in lateral mpc
   * control,horizon, meanwhile suppose kappa rate upper and lower are const in
   * mpc.
   * TODO(shijun): render the kappa ouput lower/uppper in vantage.
   */

  // When av is not AUTO_MODE, use static limit without regard to speed and
  // lateral jerk limit.
  if (autonomy_state.autonomy_state() != AutonomyStateProto::AUTO_DRIVE) {
    steering_protection_result.set_kappa_rate_upper(kappa_rate_limit_static);
    steering_protection_result.set_kappa_rate_lower(-kappa_rate_limit_static);
    for (int i = 0; i < kSControlHorizon; ++i) {
      steering_protection_result.add_kappa_output_upper(
          kappa_limit_wrt_geometry);
      steering_protection_result.add_kappa_output_lower(
          -kappa_limit_wrt_geometry);
    }
    return steering_protection_result;
  }

  // Av is AUTO_MODE.
  double kappa_output_upper = previous_kappa_cmd;
  double kappa_output_lower = previous_kappa_cmd;
  for (int i = 0; i < kSControlHorizon + 1; ++i) {
    if (i == 0) {
      kappa_output_upper += kappa_rate_limit_final * kControlInterval;
      kappa_output_lower -= kappa_rate_limit_final * kControlInterval;

      steering_protection_result.set_kappa_rate_upper(kappa_rate_limit_final);
      steering_protection_result.set_kappa_rate_lower(-kappa_rate_limit_final);
      continue;
    }

    const double published_kappa_output_upper =
        std::clamp(kappa_output_upper, -kappa_limit_wrt_geometry_and_speed,
                   kappa_limit_wrt_geometry_and_speed);
    const double published_kappa_output_lower =
        std::clamp(kappa_output_lower, -kappa_limit_wrt_geometry_and_speed,
                   kappa_limit_wrt_geometry_and_speed);

    QCHECK_GE(published_kappa_output_upper, published_kappa_output_lower)
        << " i = " << i
        << ", published_kappa_output_upper = " << published_kappa_output_upper
        << ", published_kappa_output_lower = " << published_kappa_output_lower;

    steering_protection_result.add_kappa_output_upper(
        published_kappa_output_upper);
    steering_protection_result.add_kappa_output_lower(
        published_kappa_output_lower);

    kappa_output_upper +=
        kappa_rate_limit_final * control_conf_->ts_pkmpc_controller_conf().ts();
    kappa_output_lower -=
        kappa_rate_limit_final * control_conf_->ts_pkmpc_controller_conf().ts();
  }

  return steering_protection_result;
}

bool SteeringProtection::IsProtectiveKickout(
    double av_speed,
    const boost::circular_buffer<ControlHistoryStateManager::ControlStateData>
        &control_state_cache,
    const ControllerDebugProto &controller_debug_proto,
    SteeringProtectionResult *steering_protection_result) const {
  if (!control_state_cache.full()) {
    return false;
  }

  if (!control_state_cache.back().is_under_control) {
    return false;
  }

  const int control_state_cache_size = control_state_cache.size();
  double steering_protection_kickout_time =
      std::fabs(av_speed) > Kph2Mps(5.0) ? 0.4 : 1.5;  // s
  steering_protection_kickout_time =
      std::min(steering_protection_kickout_time,
               control_state_cache_size * kControlInterval);

  const double kappa_cmd_newest = control_state_cache.back().kappa_cmd;
  const int index_step = std::clamp(
      FloorToInt(steering_protection_kickout_time / kControlInterval) + 1, 1,
      control_state_cache_size);
  const double kappa_cmd_a_certain_time_ago =
      (control_state_cache.end() - index_step)->kappa_cmd;
  const double kappa_rate_actual_mean =
      (kappa_cmd_newest - kappa_cmd_a_certain_time_ago) /
      (steering_protection_kickout_time);

  steering_protection_result->set_kappa_rate_actual_mean(
      kappa_rate_actual_mean);

  // TODO(shijun): consider steering constraint cache value.
  const bool is_kappa_rate_actual_over_limit =
      std::fabs(kappa_rate_actual_mean) >
      kRelaxFactorThreshold * steering_protection_result->kappa_rate_upper();

  // If there is no prediction, only consider actual kappa rate in past time.
  if (!controller_debug_proto.has_mpc_debug_proto() ||
      controller_debug_proto.mpc_debug_proto().s_control_mpc_result_size() !=
          kSControlHorizon) {
    return is_kappa_rate_actual_over_limit;
  }

  // Consider future kappa rate prediction.
  const auto &s_control_mpc_result =
      controller_debug_proto.mpc_debug_proto().s_control_mpc_result();
  constexpr double kPredictTime = 0.4;  // s.
  const int predict_steps =
      FloorToInt(kPredictTime / control_conf_->ts_pkmpc_controller_conf().ts());
  // Pay attention: S_control_mpc_result records kappa in ts_pk_mpc and
  // kappa_rate in tob_ts_pk_mpc.
  // TODO(yangyu/shijun): consider Pole Placement controller.
  double kappa_rate_predict_mean = 0.0;
  if (control_conf_->active_controllers()[0] ==
      ControllerConf::TS_PKMPC_CONTROLLER) {
    // Ensure denominator is not zero.
    kappa_rate_predict_mean =
        (s_control_mpc_result[predict_steps] - s_control_mpc_result[0]) /
        (predict_steps * control_conf_->ts_pkmpc_controller_conf().ts() + 1e-5);
  } else if (control_conf_->active_controllers()[0] ==
             ControllerConf::TOB_TSPKMPC_CONTROLLER) {
    double kappa_rate_predict_sum = 0.0;
    for (int i = 0; i < predict_steps; ++i) {
      kappa_rate_predict_sum += s_control_mpc_result[i];
    }
    kappa_rate_predict_mean = kappa_rate_predict_sum / predict_steps;
  }

  steering_protection_result->set_kappa_rate_predict_mean(
      kappa_rate_predict_mean);

  const bool is_kappa_rate_predict_over_limit =
      std::fabs(kappa_rate_predict_mean) >
      kRelaxFactorThreshold * steering_protection_result->kappa_rate_upper();

  const bool is_kappa_rate_actual_and_future_same_direction =
      kappa_rate_actual_mean * kappa_rate_predict_mean > 0.0;

  return is_kappa_rate_actual_over_limit && is_kappa_rate_predict_over_limit &&
         is_kappa_rate_actual_and_future_same_direction &&
         std::fabs(av_speed) > Kph2Mps(3.0);
}

void SteeringProtection::FillDebugMessage(
    const VehicleStateProto &vehicle_state,
    const SteeringProtectionResult &steering_protection_result,
    std::string *debug_msg) const {
  const double av_kappa =
      FrontWheelAngle2Kappa(vehicle_state.front_wheel_steering_angle(),
                            vehicle_geometry_params_->wheel_base());
  const double steering_rate_limitation =
      KappaRate2SteerRate(steering_protection_result.kappa_rate_upper(),
                          av_kappa, vehicle_geometry_params_->wheel_base(),
                          vehicle_drive_params_->steer_ratio());
  const double steering_rate_actual =
      KappaRate2SteerRate(steering_protection_result.kappa_rate_actual_mean(),
                          av_kappa, vehicle_geometry_params_->wheel_base(),
                          vehicle_drive_params_->steer_ratio());
  const double steering_rate_predict =
      KappaRate2SteerRate(steering_protection_result.kappa_rate_predict_mean(),
                          av_kappa, vehicle_geometry_params_->wheel_base(),
                          vehicle_drive_params_->steer_ratio());
  *debug_msg = absl::StrCat(
      "steering rate limitation is ", r2d(steering_rate_limitation),
      " deg/s when speed is at ", Mps2Kph(vehicle_state.linear_velocity()),
      " km/h and actual steering rate is ", r2d(steering_rate_actual),
      " deg/s and predict steering rate is ", r2d(steering_rate_predict),
      " deg/s.");
}

}  // namespace qcraft::control
