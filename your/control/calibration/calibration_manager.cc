#include "onboard/control/calibration/calibration_manager.h"

#include <algorithm>
#include <utility>

#include "glog/logging.h"
#include "onboard/eval/qevent.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"

namespace qcraft {
namespace control {

constexpr double kSpeedModeAccBound = 0.0;  // -0.2

absl::Status CalibrationManager::Init(
    const VehicleDriveParamsProto &vehicle_drive_params,
    const bool enable_ebrake) {
  vehicle_drive_params_ = vehicle_drive_params;
  if (vehicle_drive_params_.has_enable_calibration_v2()) {
    enable_version2_ = vehicle_drive_params_.enable_calibration_v2();
  }
  enable_ebrake_ = enable_ebrake;
  throttle_lowerbound_ = vehicle_drive_params_.throttle_deadzone();
  brake_lowerbound_ = vehicle_drive_params_.brake_deadzone();

  if (!enable_version2_) {
    if (!vehicle_drive_params_.has_calibration_table()) {
      const auto error_msg = "[Lon Calibration] Version 1.0 not parameters!";
      QLOG_EVERY_N_SEC(ERROR, 1.0) << error_msg;
      return absl::InvalidArgumentError(error_msg);
    }

    control_interpolation2D_ = std::make_unique<Interpolation2D>();
    if (enable_ebrake_) {
      ebrake_interpolation2D_ = std::make_unique<Interpolation2D>();
    }
    QLOG_EVERY_N_SEC(INFO, 1.0)
        << "[Lon Calibration] Import Calibration Version 1.0 ...";
  } else {
    if (!vehicle_drive_params_.has_calibration_table_v2() ||
        !vehicle_drive_params_.calibration_table_v2().has_idle_v_a_plf() ||
        !vehicle_drive_params_.calibration_table_v2().has_a_throttle_plf() ||
        !vehicle_drive_params_.calibration_table_v2().has_a_brake_plf()) {
      const auto error_msg = "[Lon Calibration] Version 2.0 not parameters!";
      QLOG_EVERY_N_SEC(ERROR, 1.0) << error_msg;

      return absl::InvalidArgumentError(error_msg);
    }
    QLOG_EVERY_N_SEC(INFO, 1.0)
        << "[Lon Calibration] Import Calibration Version 2.0 ...";
  }
  LoadControlCalibration();
  QLOG_EVERY_N_SEC(INFO, 1.0) << "[Lon Calibration] Load Calibration Success!";
  return absl::OkStatus();
}

void CalibrationManager::UpdateCalibrationValue(
    const double speed, const double acceleration_target,
    const Chassis::GearPosition gear_position,
    CalibrationValue *calibration_value) {
  if (!enable_version2_) {
    const double control_calibration = control_interpolation2D_->Interpolate(
        std::make_pair(speed, acceleration_target));

    double ebrake_calibration = 0.0;
    if (enable_ebrake_) {
      ebrake_calibration = ebrake_interpolation2D_->Interpolate(
          std::make_pair(speed, acceleration_target));
    }
    calibration_value_.control_calibration_value = control_calibration;
    calibration_value_.ebrake_calibration_value = ebrake_calibration;
  } else {
    constexpr double kEpsilonSpeed = 0.01;
    if (gear_position == Chassis::GEAR_REVERSE) {
      acceleration_idle_ = idle_v_a_plf_(std::min(speed, -kEpsilonSpeed));
    } else {
      acceleration_idle_ = idle_v_a_plf_(std::max(speed, kEpsilonSpeed));
    }
    acceleration_pure_ = acceleration_target - acceleration_idle_;
    calibration_value_.control_calibration_value =
        UpdateCalibrationValueV2(acceleration_pure_, speed, gear_position);
    calibration_value_.ebrake_calibration_value = 0.0;
  }
  *calibration_value = calibration_value_;
}

void CalibrationManager::ComputeLongitudinalCmd(
    const double calibration_value, CalibrationCmd *calibration_cmd) {
  if (calibration_value >= 0.0) {
    calibration_cmd_.throttle_cmd =
        calibration_value > throttle_lowerbound_ ? calibration_value : 0.0;
    calibration_cmd_.brake_cmd = 0.0;
    calibration_cmd_.ebrake_cmd = 0.0;
  } else {
    calibration_cmd_.throttle_cmd = 0.0;
    calibration_cmd_.brake_cmd =
        -calibration_value > brake_lowerbound_ ? -calibration_value : 0.0;
    const auto ebrake_value = calibration_value_.ebrake_calibration_value;
    calibration_cmd_.ebrake_cmd =
        ebrake_value < 0.0 ? std::fabs(ebrake_value) : 0.0;
  }
  *calibration_cmd = calibration_cmd_;
}

double CalibrationManager::GetIdleAcceleration() {
  if (enable_version2_) {
    return acceleration_idle_;
  }
  return kSpeedModeAccBound;
}

double CalibrationManager::GetPureAcceleration() {
  if (enable_version2_) {
    return acceleration_pure_;
  }
  return 0.0;
}

void CalibrationManager::LoadControlCalibration() {
  if (!enable_version2_) {
    if (enable_ebrake_) {
      // particular for jinlvminibus electric_brake control use
      QCHECK(
          SetInterpolation2D(vehicle_drive_params_.ebrake_calibration_table(),
                             ebrake_interpolation2D_))
          << "Faid to load ebrake calibraiton table";
      QCHECK(SetInterpolation2D(
          vehicle_drive_params_.ibs_calibration_table_when_ebrake(),
          control_interpolation2D_))
          << "Faid to load ibs_calibration_table_when_ebrake";
    } else {
      QCHECK(SetInterpolation2D(vehicle_drive_params_.calibration_table(),
                                control_interpolation2D_))
          << "Faid to load control calibraiton table without ebrake";
    }
  } else {
    idle_v_a_plf_ = PiecewiseLinearFunctionFromProto(
        vehicle_drive_params_.calibration_table_v2().idle_v_a_plf());
    a_throttle_plf_ = PiecewiseLinearFunctionFromProto(
        vehicle_drive_params_.calibration_table_v2().a_throttle_plf());
    a_brake_plf_ = PiecewiseLinearFunctionFromProto(
        vehicle_drive_params_.calibration_table_v2().a_brake_plf());
  }
}

double CalibrationManager::UpdateCalibrationValueV2(
    const double acceleration_pure, const double speed,
    Chassis::GearPosition gear_position) {
  constexpr double kEpsilon = 0.1;
  double a_gain_wrt_speed = 1.0;
  if (vehicle_drive_params_.calibration_table_v2().has_a_gain_wrt_speed_plf()) {
    const auto a_gain_wrt_speed_plf = PiecewiseLinearFunctionFromProto(
        vehicle_drive_params_.calibration_table_v2().a_gain_wrt_speed_plf());
    a_gain_wrt_speed = a_gain_wrt_speed_plf.Evaluate(std::fabs(speed));
  }
  if (acceleration_pure >= 0.0) {
    if (gear_position == Chassis::GEAR_REVERSE && speed <= kEpsilon) {
      return -a_brake_plf_(acceleration_pure);
    }
    return a_throttle_plf_(acceleration_pure * a_gain_wrt_speed);
  } else {
    if (gear_position == Chassis::GEAR_REVERSE && speed <= kEpsilon) {
      return a_throttle_plf_(acceleration_pure * a_gain_wrt_speed);
    }
    return -a_brake_plf_(acceleration_pure);
  }
}

bool CalibrationManager::SetInterpolation2D(
    const ControlCalibrationTable &calibration_table,
    const std::unique_ptr<Interpolation2D> &interpolation2D_ptr) {
  VLOG(1) << "Control calibration table loaded";
  VLOG(1) << "Control calibration table size is "
          << calibration_table.calibration_size();
  Interpolation2D::DataType xyz;
  for (const auto &calibration : calibration_table.calibration()) {
    xyz.push_back(std::make_tuple(calibration.speed(),
                                  calibration.acceleration(),
                                  calibration.command()));
  }
  return interpolation2D_ptr->Init(xyz);
}

}  // namespace control
}  // namespace qcraft
