#ifndef ONBOARD_CONTROL_OPENLOOP_CONTROL_OPENLOOP_CONTROL_H_
#define ONBOARD_CONTROL_OPENLOOP_CONTROL_OPENLOOP_CONTROL_H_

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "onboard/control/openloop_control/proto/openloop_cmd.pb.h"
#include "onboard/control/vehicle_state_interface.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace control {

class OpenloopControl {
 public:
  absl::Status Init(const std::string &config_path);

  /**
   * @description: Computer open-loop control command.
   * @param {vehicle_speed} current vehicle speed, m/s.
   * @param {steering_percentage} current steer percentage, %.
   * @param {kappa} current vehicle kappa (yawrate/speed), 1/m.
   * @param {vehicle_base} vehicle wheelbase, m.
   * @param {max_steer_angle} vehicle max wheel steer angle, rad.
   * @param {cmd} control command.
   * @param {controller_debug_proto} control debug.
   * @return {Status}
   */
  absl::Status Process(double vehicle_speed, double steer_ratio,
                       double steering_percentage, double kappa,
                       double vehicle_base, double max_steer_angle,
                       ControlCommand *cmd,
                       ControllerDebugProto *controller_debug_proto);

  absl::Status Reset();

 private:
  bool flag_checkready_ = false;

  OpenloopCmd openloop_cmd_;
  std::vector<double> vec_aperture_;
  std::vector<int> vec_duration_num_;

  double pose_vehicle_speed_ = 0.0;
  double chassis_steer_percentage_ = 0.0;
  double pose_kappa_ = 0.0;
  double pose_steer_percentage_ = 0.0;
  double param_steer_ratio_ = 15.0;
  double param_wheel_base_ = 2.0;
  double param_max_steer_ = 9.0;

  int stage_seq_ = 0;
  bool flag_brake_ = false;  // if or not reach max speed, and start to brake
  int num_seq_ = 0;  // use sequence to determine if the duration is reached

  bool flag_steer_ = false;  // if or not reach expect speed，and start to steer
  int cmd_steer_seq_ = 0;    // use sequence to compute sin(value)
  double steer_sample_period_ = 10.0;  // steer angle sample period, 10s

  double aperture_cmd_last_ = 0.0;
  double error_speed_integral_ = 0.0;
  double error_speed_last_ = 0.0;

  absl::Status LoadParameters(const std::string &config_path);
  // If steer percentage is over limit when initialization,not go to test.
  // And go to test,until to check that current steer percentage is low than
  // limit. After that Go to test,no longer check ready.
  bool IsReadyToTest();
  bool CreateCmd(ControlCommand *cmd);
  bool ProduceThrottleBrake(ControlCommand *cmd);
  bool ProduceAcceleration(ControlCommand *cmd);
  bool ProduceSteer(ControlCommand *cmd);
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_OPENLOOP_CONTROL_OPENLOOP_CONTROL_H_
