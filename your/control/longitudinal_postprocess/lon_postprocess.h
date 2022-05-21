#ifndef ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_MODULE_H_
#define ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_MODULE_H_

#include <memory>
#include <string>

#include "onboard/control/calibration/calibration_manager.h"
#include "onboard/control/closed_loop_acc/speed_mode_manager.h"
#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft::control {

// Acceleration interface define.
// https://qcraft.feishu.cn/docs/doccnc5INPjOm80l1kPCapeteNg#1Jdjfc

struct LonPostProcessInput {
  bool is_auto_mode;
  Chassis::GearPosition gear_fb;
  bool low_speed_freespace = false;
  double acc_target = 0.0;
  double acc_feedback = 0.0;
  double acc_planner = 0.0;
  double speed_feedback = 0.0;
  double speed_planner = 0.0;
  double pitch_pose = 0.0;
  double steer_wheel_angle = 0.0;
  bool is_standstill = false;
};

class LonPostProcess {
 public:
  LonPostProcess(const ControllerConf *control_config,
                 const VehicleDriveParamsProto *vehicle_drive_params);

  /**
   * @description: Longitudinal post process
   * @param acc_target {acceleration_cmd, from mpc controller}
   * @param acc_feedback {acceleration_feedback, from pose}
   * @param acc_planner {acceleration_planner, from planner}
   * @param speed_feedback {speed_feedback, from pose}
   * @param speed_planner {speed_planner, from planner}
   * @param pitch_pose {pitch, from pose, pitch < 0 ,when climb slope}
   * @param steer_wheel_angle {steer_wheel_angle, from chassis}
   * @param is_standstill {is_standstill, from controller}
   * @param gear_fb {gear_position, from chassis}
   * @param low_speed_freespace {low_speed_freespace, from planner}
   * @param autonomy_state {autonomy_state, autodrive is or not}
   */

  void Process(const LonPostProcessInput &input, ControlCommand *control_cmd,
               ControllerDebugProto *control_debug);

 private:
  // Add deceleration filter to avoid hard brake.
  double ComputeAccelerationTargetClimb(double acc_target);

  // Compute acceleration offset.
  void InitMeanFilterForSlope();
  double ComputeAccelerationOffset(double sin_slope);

  // Compute fullstop state
  bool IsFullStop(Chassis::GearPosition gear_fb, double acc_planner,
                  double speed_planner, double speed_feedback);

  // Compute speed cmd.
  double ComputeSpeedCmd(double acc_target_climb, double speed_feedback,
                         bool is_full_stop, bool low_speed_freespace);

  // Compute acceleration standstill.
  double ComputeAccelerationStandstill(Chassis::GearPosition gear_fb,
                                       double speed_feedback, bool is_full_stop,
                                       double acc_offset);

  // Compute acceleration calibration.
  double ComputeAccelerationCalibration(Chassis::GearPosition gear_fb,
                                        double steer_wheel_angle,
                                        double acc_target_climb,
                                        bool is_full_stop,
                                        double acc_standstill_climb,
                                        double acc_offset);

  double UpdateAccCalibrationWhenSteer(double steer_wheel_angle,
                                       double acc_calibration);

  // Speed mode and closed acc function.
  double ComputeAccCalibrationBySpeedMode(
      bool is_auto_mode, Chassis::GearPosition gear_fb, double speed_feedback,
      double acc_calibration, double acc_offset, double acc_target_climb);

  // Update standstill state.
  bool UpdateStandStillState(double speed_feedback, double control_period,
                             bool is_full_stop, bool is_standstill,
                             const StandStillProto &standstill_proto);

  // Longitudinal control calibration.
  double ComputeThrottleBrakeAperture(
      bool is_auto_mode, Chassis::GearPosition gear_fb, double speed_feedback,
      double acc_feedback, double steer_wheel_angle, double acc_calibration,
      bool is_full_stop);

  // Config
  const ControllerConf *control_config_ = nullptr;
  const VehicleDriveParamsProto *vehicle_params_config_ = nullptr;
  ControllerDebugProto control_debug_;

  double hard_brake_cmd_integral_ = 0.01;
  double previous_acceleration_target_ = 0.0;
  double previous_acceleration_calibration_ = 0.0;
  double previous_speed_cmd_ = 0.0;
  int standstill_counter_ = 0;
  bool is_standstill_ = false;
  bool previous_lowspeed_freespace_ = false;

  apollo::common::MeanFilter sin_slope_mean_filter_;
  std::unique_ptr<ClosedLoopAcc> speed_mode_acc_closed_loop_;
  std::unique_ptr<CalibrationManager> calibration_manager_;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_LONGITUDINAL_POSTPROCESS_MODULE_H_
