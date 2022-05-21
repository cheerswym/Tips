#ifndef ONBOARD_CONTROL_CALIBRATION_CALIBRATION_MANAGE_H_
#define ONBOARD_CONTROL_CALIBRATION_CALIBRATION_MANAGE_H_

#include <memory>

#include "absl/status/status.h"
#include "onboard/math/interpolation_2d.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {
using ControlCalibrationTable = ControlCalibrationTable;
using ControlCalibrationVersion2 = ControlCalibrationTableV2;

struct CalibrationValue {
  double control_calibration_value = 0.0;  // calibration no ebrake,-100%~100%
  double ebrake_calibration_value = 0.0;   // calibration only ebrake,-100%~100%
};
struct CalibrationCmd {
  double ebrake_cmd = 0.0;    // ebrake control command,0~100%
  double throttle_cmd = 0.0;  // throttle control command,0~100%
  double brake_cmd = 0.0;     // brake control command,0~100%
};

/**
 * @class ControlCalibration
 *
 * @brief Manage version 1.0 and 2.0 of longitudinal control calibration table.
 */
class CalibrationManager {
 public:
  /**
   * @description: Initialize ControlCalibration and Load calibration table.
   * @param vehicle_drive_params {vehicle_drive_params proto}
   * @param enable_ebrake {if or not enable ebrake function}
   * @param throttle_lowerbound {throttle deadzone value}
   * @param brake_lowerbound {brake deadzone value}
   * @return {Initialize status}
   */
  absl::Status Init(const VehicleDriveParamsProto &vehicle_drive_params,
                    bool enable_ebrake);

  /**
   * @description: Compute calibration table value according to current
   *               vehicle speed and acceleration target.
   * @param speed {current vehicle speed}
   * @param acceleration_target {target acceleration}
   * @return calibration_value {control and ebrake calibration value}
   */
  void UpdateCalibrationValue(double speed, double acceleration_target,
                              Chassis::GearPosition gear_position,
                              CalibrationValue *calibration_value);

  /**
   * @description: Compute throttle and brake command according to update
   * calibration_value from speed model function.
   * @param calibration_value {calibration_value from speed model function}
   * @return calibration_cmd {include:throttle_cmd, brake_cmd, ebrake_cmd}
   */
  void ComputeLongitudinalCmd(double calibration_value,
                              CalibrationCmd *calibration_cmd);

  /**
   * @description: Compute acceleration according to current vehicle
   *               speed, based on idling and sliding curve.
   *               and calibration 1.0: -0.2.
   * @return {acceleration}
   */

  double GetIdleAcceleration();

  /**
   * @description: Compute acceleration only drive or brake.
   * @return {acceleration}
   */

  double GetPureAcceleration();

  // Based on both Calibration Version 1.0 and Version 2.0
 private:
  bool enable_version2_ = false;  // enable calibration version 2.0
  double throttle_lowerbound_ = 0.0;
  double brake_lowerbound_ = 0.0;
  VehicleDriveParamsProto vehicle_drive_params_;
  CalibrationCmd calibration_cmd_;
  CalibrationValue calibration_value_;

  /**
   * @description: Load control calibration table version 1.0 and 2.0
   */
  void LoadControlCalibration();

  /**
   * @description: Update control calibration table value of version 2.0
   */
  double UpdateCalibrationValueV2(double acceleration_pure, double speed,
                                  Chassis::GearPosition gear_position);

  // Based on Calibration Version 1.0
 private:
  bool enable_ebrake_ = false;  // enable electronic-brake function
  std::unique_ptr<Interpolation2D> control_interpolation2D_;
  std::unique_ptr<Interpolation2D> ebrake_interpolation2D_;

  /**
   * @description: For calibration version 1.0, initialize
   *               interpolation2D function.
   * @param calibration_table {CalibrationTable proto}
   * @param calibration_interpolation2D {Interpolation2D point function}
   * @return {bool status, interpolation2D point}
   */
  bool SetInterpolation2D(
      const ControlCalibrationTable &calibration_table,
      const std::unique_ptr<Interpolation2D> &interpolation2D_ptr);

  // Based on Calibration Version 2.0
 private:
  PiecewiseLinearFunction<double> idle_v_a_plf_;
  PiecewiseLinearFunction<double> a_throttle_plf_;
  PiecewiseLinearFunction<double> a_brake_plf_;
  double acceleration_idle_ = 0.0;  // only idle or drag acceleration
  double acceleration_pure_ = 0.0;  // except drag, drive or brake acceleration
};

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_calibration_CALIBRATION_MANAGE_H_
