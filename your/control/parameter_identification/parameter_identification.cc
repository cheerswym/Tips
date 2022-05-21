#include "onboard/control/parameter_identification/parameter_identification.h"

#include <math.h>

#include <algorithm>
#include <numeric>

#include "onboard/autonomy_state/autonomy_state_util.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft::control {

namespace {

constexpr double kLowSpeedLimit = 3.0;       // m/s
constexpr double kMaxCurvatureLimit = 6e-3;  // About 1 degree for front wheel.

}  // namespace

void ParameterIdentificator::Process(
    const PoseProto& pose, const Chassis& chassis,
    const AutonomyStateProto& autonomy_state,
    ControllerDebugProto* controller_debug_proto) {
  QCHECK(controller_debug_proto != nullptr);
  SCOPED_QTRACE("ParameterIdentificator::Process");
  // Ignore these data:
  // Manual data, very low speed data and big steering data.
  const bool is_manual_mode = !IS_AUTO_DRIVE(autonomy_state.autonomy_state());
  const bool is_speed_too_low = pose.vel_body().x() < kLowSpeedLimit;
  const bool is_steering_too_big =
      std::abs(pose.curvature()) > kMaxCurvatureLimit;
  if (is_manual_mode || is_speed_too_low || is_steering_too_big) {
    pose_curvature_cache_.clear();
    return;
  }

  const auto steering_bias_identification_input_data =
      AssembleSteerBiasIdentificationInputData(pose, chassis);
  if (steering_bias_identification_input_data.has_value()) {
    const double current_steering_bias =
        CalculateSteerBias(steering_bias_identification_input_data.value());

    SteeringBiasStatistics(current_steering_bias, controller_debug_proto);
  }
}

void ParameterIdentificator::UpdateSteerDelay(
    const Chassis& chassis, const AutonomyStateProto& autonomy_state) {
  const double chassis_front_wheel_angle = SteeringPct2FrontWheelAngle(
      chassis.steering_percentage(), vehicle_drive_params_.steer_ratio(),
      vehicle_drive_params_.max_steer_angle());
  const bool is_auto_mode = IS_AUTO_DRIVE(autonomy_state.autonomy_state());

  steer_delay_.steer_delay =
      CalculateSteerDelay(is_auto_mode, chassis_front_wheel_angle);
}

std::optional<ParameterIdentificator::SteerBiasIdentificationInputData>
ParameterIdentificator::AssembleSteerBiasIdentificationInputData(
    const PoseProto& pose, const Chassis& chassis) {
  if (pose_curvature_cache_.size() < kSteeringBiasDelayCacheSize) {
    pose_curvature_cache_.push_back(pose.curvature());
    return std::nullopt;
  }
  const double pose_curvature_wrt_steering_delay =
      pose_curvature_cache_.front();
  pose_curvature_cache_.push_back(pose.curvature());
  const double front_wheel_angle = SteeringPct2FrontWheelAngle(
      chassis.steering_percentage(), vehicle_drive_params_.steer_ratio(),
      vehicle_drive_params_.max_steer_angle());

  return std::make_optional<
      ParameterIdentificator::SteerBiasIdentificationInputData>(
      {.front_wheel_angle = front_wheel_angle,
       .kappa = pose_curvature_wrt_steering_delay});
}

double ParameterIdentificator::CalculateSteerBias(
    const SteerBiasIdentificationInputData&
        steer_bias_identification_input_data) {
  // Using gradient to solve the result: steering_bias which minimum the
  // quadratic cost. J = a * x^2 + b * x + c, a > 0, x* = -b / 2a.
  // Iterative computation steering bias.

  const double current_steering_bias =
      (prev_valid_result_num_ * prev_steering_bias_ -
       (steer_bias_identification_input_data.front_wheel_angle -
        std::atan(vehicle_geometry_params_.wheel_base() *
                  steer_bias_identification_input_data.kappa))) /
      (prev_valid_result_num_ + 1);

  prev_steering_bias_ = current_steering_bias;
  ++prev_valid_result_num_;
  return current_steering_bias;
}

void ParameterIdentificator::SteeringBiasStatistics(
    double current_steering_bias,
    ControllerDebugProto* controller_debug_proto) {
  QCHECK(controller_debug_proto != nullptr);

  controller_debug_proto->mutable_parameter_identification_stat()
      ->set_stat_index(prev_valid_result_num_);
  // Pay attention to the convertion from front wheel angle bias to steering
  // angle bias.
  controller_debug_proto->mutable_parameter_identification_stat()
      ->set_steer_angle_bias(current_steering_bias *
                             vehicle_drive_params_.steer_ratio());
}

double ParameterIdentificator::CalculateSteerDelay(bool is_auto,
                                                   double steer_cmd) {
  return is_auto ? steer_delay_.steer_delay_plf.Evaluate(std::fabs(steer_cmd))
                 : steer_delay_.canbus_steer_delay;
}

BiasEstimationDebug ParameterIdentificator::EstimateSteerBias(
    const ParameterIdentificationInput& input) {
  BiasEstimationDebug bias_estimation_debug;

  const bool is_tracking_offset_threshold_satisfied =
      (std::fabs(input.heading_err) < bias_estimation_conf_.heading_err_ub() &&
       std::fabs(input.lat_error) < bias_estimation_conf_.lateral_err_ub());
  const bool is_vel_threshold_satisfied =
      (std::fabs(input.speed_measurement) > bias_estimation_conf_.vel_lb());
  const bool is_steering_satisfied =
      std::max(std::fabs(input.steer_cmd), std::fabs(input.steer_feedback)) <
      bias_estimation_conf_.steering_ub();

  const int steer_delay_cycle = FloorToInt(steer_delay_.steer_delay / ts_);

  if (input.control_history_state_mgr->GetControlStateCache().size() <
      steer_delay_cycle) {
    return bias_estimation_debug;
  }

  const auto steer_cache_iter =
      input.control_history_state_mgr->GetControlStateCache().end() - 1 -
      steer_delay_cycle;
  const double front_wheel_target = Kappa2FrontWheelAngle(
      steer_cache_iter->kappa_cmd, vehicle_geometry_params_.wheel_base());
  const double front_wheel_feedback = SteeringPct2FrontWheelAngle(
      steer_cache_iter->steer_pct_chassis, vehicle_drive_params_.steer_ratio(),
      vehicle_drive_params_.max_steer_angle());
  const double front_wheel_angle_wrt_delay = steer_cache_iter->is_under_control
                                                 ? front_wheel_target
                                                 : front_wheel_feedback;
  const double steer_bias_curr = front_wheel_angle_wrt_delay - input.steer_pose;
  calib_steering_ = (1.0 - weight_update_steer_) * calib_steering_ +
                    weight_update_steer_ * steer_bias_curr;

  bool is_bias_updated = false;
  if (is_tracking_offset_threshold_satisfied && is_vel_threshold_satisfied &&
      is_steering_satisfied) {
    is_bias_updated = true;

    // TODO(Yangyu): confirm whether using the correct weight.
    calib_heading_ = (1.0 - weight_update_heading_) * calib_heading_ +
                     weight_update_heading_ * input.heading_err;
  }

  const double kSteeringBiasThreshold = 0.005;
  const double kHeadingBiasThreshold = 0.008;
  const bool is_steer_calib_overbound =
      fabs(calib_steering_) > kSteeringBiasThreshold ? true : false;
  const bool is_heading_calib_overbound =
      fabs(calib_heading_) > kHeadingBiasThreshold ? true : false;

  const double calib_steering0_ub = bias_estimation_conf_.calib_steering0_ub();
  const double calib_heading0_ub = bias_estimation_conf_.calib_heading0_ub();
  calib_steering_ =
      std::clamp(calib_steering_, -calib_steering0_ub, calib_steering0_ub);
  calib_heading_ =
      std::clamp(calib_heading_, -calib_heading0_ub, calib_heading0_ub);
  bias_estimation_debug.set_steer_bias(calib_steering_ *
                                       vehicle_drive_params_.steer_ratio());
  bias_estimation_debug.set_heading_bias(r2d(calib_heading_));
  bias_estimation_debug.set_is_bias_updated(is_bias_updated);
  bias_estimation_debug.set_is_steer_calib_overbound(is_steer_calib_overbound);
  bias_estimation_debug.set_is_heading_calib_overbound(
      is_heading_calib_overbound);
  bias_estimation_debug.set_steer_bias_curr(
      steer_bias_curr * vehicle_drive_params_.steer_ratio());
  bias_estimation_debug.set_heading_bias_curr(r2d(input.heading_err));
  return bias_estimation_debug;
}

void ParameterIdentificator::InitEstimation(
    const ControllerConf& control_conf) {
  bias_estimation_conf_ = control_conf.bias_estimation_conf();

  ts_ = control_conf.control_period();

  calib_steering_ = vehicle_drive_params_.steering_angle_bias() /
                    vehicle_drive_params_.steer_ratio();
  weight_update_steer_ = bias_estimation_conf_.weight_on_steering() * ts_;
  weight_update_heading_ = bias_estimation_conf_.weight_on_heading() * ts_;
  steer_delay_.InitConf(control_conf);
}

}  // namespace qcraft::control
