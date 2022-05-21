#include "onboard/control/openloop_control/openloop_control.h"

#include <utility>

#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace control {

namespace openloop {
constexpr double kSpeedMaxLimit = 20.0;        // m/s, is maxest speed of car
constexpr double kSteerUpperLimit = 50.0;      // %, init max steer of safe test
constexpr double kFinishBrakeAperture = 10.0;  //%, when finish,so brake to stop
constexpr double kConstSpeedTime = 1.0;        // s, keep speed duration 1s.
constexpr double kFixedDifference = 1.0;  // s, difference threshold with speed.
}  // namespace openloop

absl::Status OpenloopControl::Init(const std::string &config_path) {
  if (LoadParameters(config_path) != absl::OkStatus()) {
    const auto error_msg = "[OpenLoop_Control] Load parameters is failed!";
    QLOG(ERROR) << error_msg;
    return absl::InvalidArgumentError(error_msg);
  }
  QLOG(INFO) << "[OpenLoop_Control] Init is OK!";
  return absl::OkStatus();
}

absl::Status OpenloopControl::Process(
    double vehicle_speed, double steer_ratio, double steering_percentage,
    double vehicle_base, double max_steer_angle, double kappa,
    ControlCommand *cmd, ControllerDebugProto *controller_debug_proto) {
  CHECK_NOTNULL(cmd);
  CHECK_NOTNULL(controller_debug_proto);

  // Computer vehicle velocity from pose
  pose_vehicle_speed_ = vehicle_speed;

  // Computer steer_percentage
  chassis_steer_percentage_ = steering_percentage;

  // Check vehicle state can start openloop control?
  if (!IsReadyToTest()) {
    const auto error_msg = "[OpenLoop_Control] Check Ready is failed!";
    QLOG_EVERY_N_SEC(ERROR, 1.0) << error_msg;
    return absl::InvalidArgumentError(error_msg);
  }

  pose_kappa_ = kappa;
  param_steer_ratio_ = steer_ratio;
  param_wheel_base_ = vehicle_base;
  param_max_steer_ = max_steer_angle;

  if (!CreateCmd(cmd)) {
    const auto error_msg = "[OpenLoop_Control] Create Cmd is failed!";
    QLOG_EVERY_N_SEC(ERROR, 1.0) << error_msg;

    return absl::InvalidArgumentError(error_msg);
  }

  return absl::OkStatus();
}

absl::Status OpenloopControl::Reset() {
  flag_checkready_ = false;
  flag_brake_ = false;
  flag_steer_ = false;
  cmd_steer_seq_ = 0;
  num_seq_ = 0;
  stage_seq_ = 0;
  aperture_cmd_last_ = 0.0;
  error_speed_integral_ = 0.0;
  error_speed_last_ = 0.0;
  return absl::OkStatus();
}

absl::Status OpenloopControl::LoadParameters(const std::string &config_path) {
  // Load proto config;
  QLOG(INFO) << "config_path: " << config_path;

  if (!file_util::TextFileToProto(config_path, &openloop_cmd_)) {
    const auto error_msg = "[OpenLoop_Control] failed to parse proto file!";
    QLOG(ERROR) << error_msg;
    return absl::InvalidArgumentError(error_msg);
  }

  if (openloop_cmd_.throttle_brake_cmd().empty()) {
    const auto error_msg = "[OpenLoop_Control] lacks aperture or times!";
    QLOG(ERROR) << error_msg;
    return absl::InvalidArgumentError(error_msg);
  }

  // Proto config to throttle and brake cmd
  for (int i = 0; i < openloop_cmd_.throttle_brake_cmd_size(); ++i) {
    vec_aperture_.emplace_back(
        openloop_cmd_.throttle_brake_cmd().Get(i).aperture());
    int duration_num =
        openloop_cmd_.throttle_brake_cmd().Get(i).duration_time() /
        kControlInterval;
    vec_duration_num_.emplace_back(duration_num);
  }

  steer_sample_period_ = openloop_cmd_.sample_period_steer_cmd();

  return absl::OkStatus();
}

bool OpenloopControl::IsReadyToTest() {
  if (!flag_checkready_) {
    if (fabs(chassis_steer_percentage_) > openloop::kSteerUpperLimit) {
      return false;
    }
  }
  flag_checkready_ = true;
  return true;
}

bool OpenloopControl::CreateCmd(ControlCommand *cmd) {
  bool state = false;
  switch (openloop_cmd_.testtype()) {
    case OpenloopCmd::THROTTLEBRAKE:
      state = ProduceThrottleBrake(cmd);
      break;
    case OpenloopCmd::ACCELERATION:
      state = ProduceAcceleration(cmd);
      break;
    case OpenloopCmd::STEER:
      state = ProduceSteer(cmd);
      break;
    default:
      break;
  }
  return state;
}

bool OpenloopControl::ProduceThrottleBrake(ControlCommand *cmd) {
  cmd->set_steering_target(0.0);
  if (openloop_cmd_.enbale_inverse_gear()) {
    cmd->set_gear_location(Chassis::GEAR_REVERSE);
  } else {
    cmd->set_gear_location(Chassis::GEAR_DRIVE);
  }
  if (stage_seq_ >= vec_aperture_.size()) {
    cmd->set_throttle(0.0);
    cmd->set_brake(openloop::kFinishBrakeAperture);  // until to velocity = 0
    QLOG_EVERY_N_SEC(INFO, 1.0) << "[OpenLoop_Control] is Finished!";
    return true;
  }
  // [500,500,500,500,500]、[0.1,0.2,0.3,-0.2,-0.1]
  double aperture = vec_aperture_[stage_seq_];
  int duration_num = vec_duration_num_[stage_seq_];

  if (aperture >= 0.0) {
    cmd->set_throttle(aperture);
    cmd->set_brake(0.0);
  } else {
    cmd->set_throttle(0.0);
    cmd->set_brake(fabs(aperture));
  }

  num_seq_++;
  if (num_seq_ >= duration_num) {
    stage_seq_++;
    num_seq_ = 0;
  }
  QLOG_EVERY_N_SEC(INFO, 0.5)
      << "[OpenLoop Control]: Throttle-Brake: "
      << "stage_seq: " << stage_seq_ << "  "
      << "duration_time: " << num_seq_ * kControlInterval;
  return true;
}

bool OpenloopControl::ProduceAcceleration(ControlCommand *cmd) {
  cmd->set_steering_target(0.0);
  if (openloop_cmd_.enbale_inverse_gear()) {
    cmd->set_gear_location(Chassis::GEAR_REVERSE);
  } else {
    cmd->set_gear_location(Chassis::GEAR_DRIVE);
  }
  if (fabs(pose_vehicle_speed_) >= openloop::kSpeedMaxLimit) {
    QLOG_EVERY_N_SEC(INFO, 1.0)
        << "[OpenLoop_Control] Speed is over kSpeedMaxLimit!";
    flag_brake_ = true;
  }
  if (flag_brake_) {
    cmd->set_acceleration(openloop_cmd_.brake_acceleration_cmd());
  } else {
    cmd->set_acceleration(openloop_cmd_.accel_acceleration_cmd());
  }

  return true;
}

bool OpenloopControl::ProduceSteer(ControlCommand *cmd) {
  // Check vehicle speed is or not reach target speed.
  const int duration_num = openloop::kConstSpeedTime / kControlInterval;
  if (fabs(pose_vehicle_speed_ - openloop_cmd_.max_speed_cmd()) <=
      openloop::kFixedDifference) {
    num_seq_++;
    if (num_seq_ >= duration_num) {
      flag_steer_ = true;
      num_seq_ = 0;
    }
  }

  // Compute error and anti-windup of error integral
  double error_speed = openloop_cmd_.max_speed_cmd() - pose_vehicle_speed_;
  if (fabs(error_speed_integral_) <
          openloop_cmd_.pid_config().speed_integral_limit() ||
      error_speed_integral_ * error_speed < 0.0) {
    error_speed_integral_ += error_speed;
  }
  // Pid control for const speed
  double aperture_cmd = openloop_cmd_.pid_config().speed_kp() * error_speed +
                        openloop_cmd_.pid_config().speed_ki() *
                            error_speed_integral_ * kControlInterval +
                        openloop_cmd_.pid_config().speed_kd() *
                            (error_speed - error_speed_last_) /
                            kControlInterval;
  error_speed_last_ = error_speed;

  // Computer aperture constraint ,avoid vehicle start too hard.
  const double delta_aperture_limit =
      kControlInterval * openloop_cmd_.pid_config().aperture_rate_limit();

  aperture_cmd =
      std::clamp(aperture_cmd, aperture_cmd_last_ - delta_aperture_limit,
                 aperture_cmd_last_ + delta_aperture_limit);
  aperture_cmd =
      std::clamp(aperture_cmd, openloop_cmd_.pid_config().aperture_min(),
                 openloop_cmd_.pid_config().aperture_max());
  aperture_cmd_last_ = aperture_cmd;

  // Create cmd
  cmd->set_gear_location(Chassis::GEAR_DRIVE);
  if (aperture_cmd > 0.0) {
    cmd->set_throttle(aperture_cmd);
    cmd->set_brake(0.0);
  } else {
    cmd->set_throttle(0.0);
    cmd->set_brake(aperture_cmd);
  }
  // Start steer, when reach target speed
  if (flag_steer_) {
    const double sample_frequency =
        kControlInterval / steer_sample_period_ * 2.0 * M_PI;  // T = 10s;
    const double cmd_steer_percentage =
        openloop_cmd_.max_steer_percentage_cmd() *
        std::sin(cmd_steer_seq_ * sample_frequency);

    const double cmd_steer_angle =
        cmd_steer_percentage / 100.0 * param_max_steer_;
    const double control_kappa = SteerAngle2Kappa(
        cmd_steer_angle, param_wheel_base_, param_steer_ratio_);

    // Computer vehicle pose steer percentage
    pose_steer_percentage_ = Kappa2SteerPercentage(
        pose_kappa_, param_wheel_base_, param_steer_ratio_, param_max_steer_);

    cmd_steer_seq_++;
    cmd->set_steering_target(cmd_steer_percentage);
    cmd->set_curvature(control_kappa);
  }

  return true;
}
}  // namespace control
}  // namespace qcraft
