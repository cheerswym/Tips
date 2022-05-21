#include "onboard/control/longitudinal_postprocess/lon_postprocess.h"

#include <algorithm>
#include <cmath>

#include "onboard/control/control_defs.h"
#include "onboard/global/car_common.h"

namespace qcraft::control {

LonPostProcess::LonPostProcess(
    const ControllerConf *control_config,
    const VehicleDriveParamsProto *vehicle_drive_params) {
  control_config_ = control_config;
  vehicle_params_config_ = vehicle_drive_params;

  InitMeanFilterForSlope();

  if (control_config_->enable_speed_mode_manager()) {
    speed_mode_acc_closed_loop_ = std::make_unique<ClosedLoopAcc>();
    speed_mode_acc_closed_loop_->InitClosedLoopAcc(
        *control_config_, vehicle_params_config_->throttle_deadzone(),
        vehicle_params_config_->brake_deadzone());
  }

  calibration_manager_ = std::make_unique<CalibrationManager>();
  QCHECK_OK(calibration_manager_->Init(*vehicle_drive_params,
                                       control_config_->ebrake_enable()));
  LOG(INFO) << "[Lon postprocess] Init is Success!";
}

void LonPostProcess::Process(const LonPostProcessInput &input,
                             ControlCommand *control_cmd,
                             ControllerDebugProto *control_debug) {
  // Comfort deacceleration(from acc_mpc, when acc < -1 m/s2).
  const double acc_target_climb =
      ComputeAccelerationTargetClimb(input.acc_target);
  previous_acceleration_target_ = acc_target_climb;

  // Slope acceleration.
  const double sin_slope = -std::sin(input.pitch_pose);
  const double acc_offset = ComputeAccelerationOffset(sin_slope);

  const bool is_full_stop =
      IsFullStop(input.gear_fb, input.acc_planner, input.speed_planner,
                 input.speed_feedback);

  // Compute speed cmd.
  double speed_cmd = ComputeSpeedCmd(acc_target_climb, input.speed_feedback,
                                     is_full_stop, input.low_speed_freespace);
  constexpr double kMaxSpeed = 4.17;   // 15km/h
  constexpr double kMinSpeed = -4.17;  // -15km/h
  speed_cmd = std::clamp(speed_cmd, kMinSpeed, kMaxSpeed);
  previous_speed_cmd_ = speed_cmd;
  previous_lowspeed_freespace_ = input.low_speed_freespace;

  // Comfort acc_standstill when fullstop.
  is_standstill_ = input.is_standstill;
  const double acc_standstill_climb = ComputeAccelerationStandstill(
      input.gear_fb, input.speed_feedback, is_full_stop, acc_offset);

  double acc_calibration = ComputeAccelerationCalibration(
      input.gear_fb, input.steer_wheel_angle, acc_target_climb, is_full_stop,
      acc_standstill_climb, acc_offset);

  if (control_config_->enable_speed_mode_manager()) {
    const double acc_calibration_speedmode = ComputeAccCalibrationBySpeedMode(
        input.is_auto_mode, input.gear_fb, input.speed_feedback,
        acc_calibration, acc_offset, acc_target_climb);
    acc_calibration = acc_calibration_speedmode;
  }
  previous_acceleration_calibration_ = acc_calibration;

  // Pedal aperture, -100~100%
  const double calibration_aperture = ComputeThrottleBrakeAperture(
      input.is_auto_mode, input.gear_fb, input.speed_feedback,
      input.acc_feedback, input.steer_wheel_angle, acc_calibration,
      is_full_stop);

  // Speed interface.
  control_cmd->set_speed(speed_cmd);

  // Acceleration interface.
  control_cmd->set_acceleration_offset(acc_offset);
  control_cmd->set_acceleration_calibration(acc_calibration);

  if (IsOnboardMode()) {
    double acc_cmd_output = acc_target_climb;
    double acc_calibration_output = acc_calibration;

    const auto curr_speed_mode =
        control_debug_.speed_mode_debug_proto().curr_speed_mode();

    constexpr double kEpsilonAcc = 0.01;

    // Use speed mode to limit acc.
    if (control_config_->enable_speed_mode_manager()) {
      const bool negative_acc_output =
          (input.gear_fb == Chassis::GEAR_REVERSE &&
           curr_speed_mode == qcraft::SpeedMode::ACC_MODE) ||
          (input.gear_fb == Chassis::GEAR_DRIVE &&
           curr_speed_mode == qcraft::SpeedMode::DEC_MODE);
      acc_cmd_output = negative_acc_output
                           ? std::min(acc_cmd_output, -kEpsilonAcc)
                           : std::max(acc_cmd_output, kEpsilonAcc);
      acc_calibration_output =
          negative_acc_output ? std::min(acc_calibration_output, -kEpsilonAcc)
                              : std::max(acc_calibration_output, kEpsilonAcc);
    }

    const int acc_interface_type = control_config_->acceleration_interface();
    switch (acc_interface_type) {
      case AccelerationInterface::CLOSEDLOOP:
        control_cmd->set_acceleration(acc_cmd_output);
        break;
      case AccelerationInterface::OPENEDLOOP:
        control_cmd->set_acceleration(acc_calibration_output);
        break;
      case AccelerationInterface::CLOSEDLOOP_WITH_PARKING_COMPENSATION:
        if (is_full_stop) {
          control_cmd->set_acceleration(acc_standstill_climb);
        } else {
          control_cmd->set_acceleration(acc_cmd_output);
        }
        break;
      default:
        control_cmd->set_acceleration(acc_target_climb);
    }
  } else {
    control_cmd->set_acceleration(acc_target_climb);
  }

  // Throttle and brake interface.
  CalibrationCmd throttle_brake;
  calibration_manager_->ComputeLongitudinalCmd(calibration_aperture,
                                               &throttle_brake);
  control_cmd->set_throttle(throttle_brake.throttle_cmd);
  control_cmd->set_brake(throttle_brake.brake_cmd);
  control_cmd->set_e_brake(throttle_brake.ebrake_cmd);

  // Update control command.
  auto mpc_debug = control_cmd->mutable_debug()->mutable_simple_mpc_debug();
  mpc_debug->set_is_full_stop(is_full_stop);
  mpc_debug->set_acceleration_cmd(input.acc_target);
  mpc_debug->set_acceleration_cmd_closeloop(input.acc_target);
  mpc_debug->set_calibration_value(calibration_aperture);

  // Update control debug.
  control_debug->mutable_speed_mode_debug_proto()->CopyFrom(
      control_debug_.speed_mode_debug_proto());
  control_debug->set_sin_slope(sin_slope);
  control_debug->set_acceleration_offset(acc_offset);
  control_debug->mutable_speed_mode_debug_proto()->set_is_standstill(
      is_standstill_);
  control_debug->mutable_speed_mode_debug_proto()->set_standstill_counter(
      standstill_counter_);

  const double acceleration_idle = calibration_manager_->GetIdleAcceleration();
  const double acceleration_pure = calibration_manager_->GetPureAcceleration();
  control_debug->mutable_calibration_debug_proto()->set_acceleration_idle(
      acceleration_idle);
  control_debug->mutable_calibration_debug_proto()->set_acceleration_pure(
      acceleration_pure);
}

void LonPostProcess::InitMeanFilterForSlope() {
  // Mean filter for sin(slope).
  const int kSinSlopeFilterWindowSize = 120;
  sin_slope_mean_filter_ =
      apollo::common::MeanFilter(kSinSlopeFilterWindowSize);
}

// Add deceleration filter(slow down for deceleration) to avoid hard brake.
double LonPostProcess::ComputeAccelerationTargetClimb(double acc_target) {
  constexpr double kHardBrakeFilterThreshold = -1.0;  // m/s^2.
  constexpr double kHardBrakeCmdIntegralRatio = 1.07;

  double acc_target_comfort = 0.0;
  if (acc_target < kHardBrakeFilterThreshold) {
    hard_brake_cmd_integral_ *= kHardBrakeCmdIntegralRatio;
    hard_brake_cmd_integral_ = std::clamp(
        hard_brake_cmd_integral_, 0.0, kGravitationalAcceleration);  // max:1g
    acc_target_comfort = std::clamp(
        acc_target, previous_acceleration_target_ - hard_brake_cmd_integral_,
        kHardBrakeFilterThreshold);
  } else {
    constexpr double kInitAccelRate = 1.0;  // m/s^3, acceleration rate.
    hard_brake_cmd_integral_ = kInitAccelRate * kControlInterval;
    acc_target_comfort = acc_target;
  }
  return acc_target_comfort;
}

double LonPostProcess::ComputeAccelerationOffset(double sin_slope) {
  // Estimate acceleration offset based on pose pitch.

  const double sin_slope_smooth = sin_slope_mean_filter_.Update(
      std::clamp(sin_slope, -kSinSlopeLimit, kSinSlopeLimit));

  const double acc_offset = sin_slope_smooth * kGravitationalAcceleration;
  return acc_offset;
}

// If or not entering control full stop conditions.
bool LonPostProcess::IsFullStop(Chassis::GearPosition gear_fb,
                                double acc_planner, double speed_planner,
                                double speed_feedback) {
  if (gear_fb == Chassis::GEAR_DRIVE && acc_planner > 0.0) {
    return false;
  }
  if (gear_fb == Chassis::GEAR_REVERSE && acc_planner < 0.0) {
    return false;
  }
  const auto full_stop_condition = control_config_->full_stop_condition();
  if (std::abs(speed_planner) >
          full_stop_condition.abs_planner_speed_upperlimit() ||
      std::abs(speed_feedback) >
          full_stop_condition.abs_linear_speed_upperlimit()) {
    return false;
  }
  return true;
}

// Compute speed cmd.
double LonPostProcess::ComputeSpeedCmd(double acc_target_climb,
                                       double speed_feedback, bool is_full_stop,
                                       bool low_speed_freespace) {
  if (is_full_stop && std::fabs(speed_feedback) < 0.01) return 0.0;
  if (low_speed_freespace != previous_lowspeed_freespace_) {
    return speed_feedback;
  }
  return previous_speed_cmd_ + acc_target_climb * kControlInterval;
}

// Compute acceleration standstill.
double LonPostProcess::ComputeAccelerationStandstill(
    Chassis::GearPosition gear_fb, double speed_feedback, bool is_full_stop,
    double acc_offset) {
  const auto full_stop_condition = control_config_->full_stop_condition();
  double acc_standstill_climb = 0.0;
  if (is_full_stop) {
    // First fullstop, when car stop.
    double acc_standstill = full_stop_condition.standstill_acceleration();  // -
    double dec_cmd_filter = kFullStopBrakeCmdIntegral;

    // Second fullstop, when car stop, increase brake.
    // Update standstill_acc and dec_cmd_filter.
    if (std::fabs(speed_feedback) < kStopSpeed) {
      constexpr double kIncreaseBrakeRate = 0.4;  // Increase 0.4 m/s^3
      acc_standstill =
          std::min(acc_standstill,
                   full_stop_condition.lockdown_acceleration());  // -
      dec_cmd_filter = kIncreaseBrakeRate * kFullStopBrakeCmdIntegral;
    }

    if (gear_fb == Chassis::GEAR_REVERSE) {
      acc_standstill = -acc_standstill - std::fabs(acc_offset);
      acc_standstill_climb = std::min(
          previous_acceleration_calibration_ + dec_cmd_filter, acc_standstill);
    } else {
      acc_standstill += -std::fabs(acc_offset);
      acc_standstill_climb = std::max(
          previous_acceleration_calibration_ - dec_cmd_filter, acc_standstill);
    }
  }

  is_standstill_ = UpdateStandStillState(
      speed_feedback, control_config_->control_period(), is_full_stop,
      is_standstill_, control_config_->standstill_condition());

  return acc_standstill_climb;
}

bool LonPostProcess::UpdateStandStillState(
    double speed_measurement, double control_period, bool is_full_stop,
    bool is_standstill, const StandStillProto &standstill_proto) {
  const int standstill_counter_threshold =
      FloorToInt(standstill_proto.standstill_time_th() / control_period);
  if (std::fabs(speed_measurement) > standstill_proto.vel_move_th() ||
      !(is_full_stop)) {
    standstill_counter_ = 0;
    return false;
  }
  if (!is_standstill) {
    if (std::fabs(speed_measurement) < standstill_proto.vel_standstill_th()) {
      standstill_counter_ =
          std::min(standstill_counter_ + 1, standstill_counter_threshold);
      if (standstill_counter_ == standstill_counter_threshold) {
        return true;
      }
    } else {
      standstill_counter_ = std::max(standstill_counter_ - 1, 0);
    }
  }
  return is_standstill;
}

// Compute acceleration calibration.
double LonPostProcess::ComputeAccelerationCalibration(
    Chassis::GearPosition gear_fb, double steer_wheel_angle,
    double acc_target_climb, bool is_full_stop, double acc_standstill_climb,
    double acc_offset) {
  double acc_calibration = 0.0;

  if (is_full_stop) {
    if (gear_fb == Chassis::GEAR_REVERSE) {
      acc_calibration = std::max(acc_target_climb, acc_standstill_climb);
    } else {
      acc_calibration = std::min(acc_target_climb, acc_standstill_climb);
    }
  } else {
    acc_calibration = acc_target_climb + acc_offset;
  }

  CHECK(std::fabs(acc_calibration) < 10.0)
      << "Abs of acc_calibration value is too big.";

  acc_calibration =
      UpdateAccCalibrationWhenSteer(steer_wheel_angle, acc_calibration);

  return acc_calibration;
}

double LonPostProcess::UpdateAccCalibrationWhenSteer(double steer_wheel_angle,
                                                     double acc_calibration) {
  double ratio_of_RAC_speed_over_linear_speed = 1.0;
  switch (vehicle_params_config_->wheel_drive_mode()) {
    case FRONT_WHEEL_DRIVE:
      ratio_of_RAC_speed_over_linear_speed = std::cos(steer_wheel_angle);
    case REAR_WHEEL_DRIVE:
      ratio_of_RAC_speed_over_linear_speed = 1.0;
    case FOUR_WHEEL_DRIVE:
      // TODO(Zhichao): need to figure out what four or all wheel drive's.
      // chassis speed stands for.
      ratio_of_RAC_speed_over_linear_speed = 1.0;
  }
  acc_calibration /= ratio_of_RAC_speed_over_linear_speed;

  return acc_calibration;
}

double LonPostProcess::ComputeAccCalibrationBySpeedMode(
    bool is_auto_mode, Chassis::GearPosition gear_fb, double speed_feedback,
    double acc_calibration, double acc_offset, double acc_target_climb) {
  // Update acc by speed mode.
  if (gear_fb == Chassis::GEAR_REVERSE) {
    acc_calibration = -acc_calibration;
    acc_target_climb = -acc_target_climb;
    speed_feedback = -speed_feedback;
  }

  const double acceleration_calibration_speedmode =
      speed_mode_acc_closed_loop_->UpdateAccCommand(
          is_auto_mode, acc_calibration, acc_target_climb, speed_feedback,
          acc_offset, &control_debug_);

  return gear_fb == Chassis::GEAR_REVERSE ? -acceleration_calibration_speedmode
                                          : acceleration_calibration_speedmode;
}

double LonPostProcess::ComputeThrottleBrakeAperture(
    bool is_auto_mode, Chassis::GearPosition gear_fb, double speed_feedback,
    double acc_feedback, double steer_wheel_angle, double acc_calibration,
    bool is_full_stop) {
  CalibrationValue calibration_value;
  calibration_manager_->UpdateCalibrationValue(speed_feedback, acc_calibration,
                                               gear_fb, &calibration_value);
  double calibration_aperture =
      calibration_value.control_calibration_value;  // -100~100%

  if (control_config_->closed_loop_acc_conf().enable_closed_loop_acc() &&
      control_config_->enable_speed_mode_manager()) {
    const auto steer_rad_abs = std::abs(std::atan(steer_wheel_angle));
    if (gear_fb == Chassis::GEAR_REVERSE) {
      acc_feedback = -acc_feedback;
    }
    const double calibration_aperture_closedloop =
        speed_mode_acc_closed_loop_->UpdateCalibrationCmd(
            is_auto_mode, is_full_stop, calibration_aperture, acc_feedback,
            steer_rad_abs, &control_debug_);
    calibration_aperture = calibration_aperture_closedloop;
  }
  return calibration_aperture;
}

}  // namespace qcraft::control
