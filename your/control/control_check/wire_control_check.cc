#include "onboard/control/control_check/wire_control_check.h"

#include <algorithm>
#include <cmath>

#include "onboard/control/controllers/controller_util.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace control {

WireControlChecker::WireControlChecker(const double control_period,
                                       const double steer_delay) {
  config_.control_period = control_period;  // control period
  config_.steer_delay = steer_delay;        // steer_delay_time

  // Check hard throttle
  config_.throttle_fb_time = 0.1;    // continuous check time when hard throttle
  config_.throttle_delay_low = 0.3;  // throttle_delay_low_time
  config_.throttle_delay_high = 1.5;  // throttle_delay_high_time
  config_.acc_hard_threshold = 2.5;   // max feeback acceleration threshold
  config_.acc_cmd_threshold = 1.75;   // max cmd acceleration threshold

  // Check steer protect
  config_.steer_mean_time = 0.4;             // mean time of kappa rate pose
  config_.steer_kappa_threshold = 0.02;      // kappa error protect threshold
  config_.steer_kapparate_threshold = 0.02;  // kappa rate error threshold
  config_.window_mean_filter = 5;            // kappa mean filter window

  QCHECK_GT(config_.control_period, 0.0)
      << "[WireControlChecker] Invalid control period "
      << config_.control_period;

  QCHECK_GT(config_.throttle_fb_time, 0.0)
      << "[WireControlChecker] Invalid throttle fb time "
      << config_.throttle_fb_time;
  QCHECK_GT(config_.throttle_delay_high, config_.throttle_delay_low)
      << "[WireControlChecker] Invalid delay vector time ";

  QCHECK_GT(config_.steer_mean_time, 0.0)
      << "[WireControlChecker] Invalid steer mean time "
      << config_.steer_mean_time;

  // Check hard throttle protect config
  num_throttle_fb_ =
      FloorToInt(config_.throttle_fb_time / config_.control_period);
  num_throttle_cmd_ =
      FloorToInt((config_.throttle_delay_high - config_.throttle_delay_low) /
                 config_.control_period);

  acceleration_fb_cache_.resize(num_throttle_fb_);
  acceleration_cmd_cache_.resize(num_throttle_cmd_);

  // Check steer protect config
  num_steer_mean_ =
      FloorToInt(config_.steer_mean_time / config_.control_period);  // 0.4s
  num_steer_delay_ =
      FloorToInt(config_.steer_delay / config_.control_period);  // 0.5s
  num_steer_cmd_ = 2.0 * num_steer_mean_ + num_steer_delay_;     // 1.3s

  kapparate_fb_cache_.resize(num_steer_mean_);
  kapparate_cmd_cache_.resize(num_steer_cmd_);
  kappa_cmd_cache_.resize(num_steer_delay_);

  kappa_cmd_filter_ =
      std::make_unique<apollo::common::MeanFilter>(config_.window_mean_filter);
  kappa_fb_filter_ =
      std::make_unique<apollo::common::MeanFilter>(config_.window_mean_filter);
}

WireControlChecker::~WireControlChecker() {
  gear_fb_last_ = Chassis::GEAR_NONE;

  // Reset check hard throttle protect status
  acceleration_fb_cache_.clear();
  acceleration_cmd_cache_.clear();

  // Reset check steer protect status
  kapparate_fb_cache_.clear();
  kapparate_cmd_cache_.clear();
  kappa_cmd_cache_.clear();
}

bool WireControlChecker::CheckProc(AutonomyStateProto_State autonomy_state,
                                   double acceleration_cmd,
                                   double acceleration_fb, double kappa_cmd,
                                   double kappa_fb, double speed_fb,
                                   Chassis::GearPosition gear_fb) {
  bool throttle_abnormal = false;
  bool steer_abnormal = false;
  // Computer kappa rate of feedback and command.
  const double kappa_cmd_smooth = kappa_cmd_filter_->Update(kappa_cmd);
  const double kappa_fb_smooth = kappa_fb_filter_->Update(kappa_fb);
  const double kappa_rate_fb =
      (kappa_fb_smooth - kappa_fb_smooth_last_) / config_.control_period;
  const double kappa_rate_cmd =
      (kappa_cmd_smooth - kappa_cmd_smooth_last_) / config_.control_period;
  kappa_fb_smooth_last_ = kappa_fb_smooth;
  kappa_cmd_smooth_last_ = kappa_cmd_smooth;

  // Change gear, or not auto drive mode ,and reset cache data.
  if (UpdateGear(gear_fb) || autonomy_state != AutonomyStateProto::AUTO_DRIVE) {
    ResetThrottle();
    ResetSteer();
    first_run_ = true;
  } else {
    // Init cache data
    if (first_run_) {
      first_run_ = false;
      acceleration_cmd_cache_.assign(num_throttle_cmd_, acceleration_cmd);
      acceleration_fb_cache_.assign(num_throttle_fb_, acceleration_fb);
      kapparate_cmd_cache_.assign(num_steer_cmd_, kappa_rate_cmd);
      kapparate_fb_cache_.assign(num_steer_mean_, kappa_rate_fb);
      kappa_cmd_cache_.assign(num_steer_delay_, kappa_cmd);
    }
    // Check steer
    steer_abnormal = IsSteeringFailure(kappa_cmd, kappa_fb, kappa_rate_cmd,
                                       kappa_rate_fb, speed_fb);
    if (steer_abnormal) {
      QLOG_EVERY_N_SEC(ERROR, 1.0)
          << "[WireControl] Check safe steer is failed!";
    }
    // Check throttle
    throttle_abnormal =
        DoesAccelerateTooHard(acceleration_cmd, acceleration_fb);
    if (throttle_abnormal) {
      QLOG_EVERY_N_SEC(ERROR, 1.0)
          << "[WireControl] Check comfort throttle is failed!";
    }
  }

  debug_.set_throttle_abnormal(throttle_abnormal);
  debug_.set_steer_abnormal(steer_abnormal);

  return throttle_abnormal || steer_abnormal;
}

bool WireControlChecker::UpdateGear(Chassis::GearPosition gear_fb) {
  if (gear_fb != gear_fb_last_) {
    gear_fb_last_ = gear_fb;
    return true;
  }
  return false;
}

bool WireControlChecker::DoesAccelerateTooHard(double acceleration_cmd,
                                               double acceleration_fb) {
  acceleration_fb_cache_.push_back(acceleration_fb);

  const auto min_acc_fb = std::min_element(std::begin(acceleration_fb_cache_),
                                           std::end(acceleration_fb_cache_));
  acceleration_cmd_cache_.push_front(acceleration_cmd);

  const auto max_acc_cmd =
      std::max_element(std::end(acceleration_cmd_cache_) - num_throttle_cmd_,
                       std::end(acceleration_cmd_cache_));
  // Debug
  debug_.set_max_acceleration_cmd(*max_acc_cmd);
  debug_.set_min_acceleration_fb(*min_acc_fb);

  if (*min_acc_fb >= config_.acc_hard_threshold &&
      *max_acc_cmd <= config_.acc_cmd_threshold) {
    return true;  // Abormal status: hard throttle
  }

  return false;  // Normal status: not hard throttle
}

void WireControlChecker::ResetThrottle() {
  acceleration_fb_cache_.assign(num_throttle_fb_, 0.0);
  acceleration_cmd_cache_.assign(num_throttle_cmd_, 0.0);
}

bool WireControlChecker::IsSteeringFailure(double kappa_cmd, double kappa_fb,
                                           double kappa_rate_cmd,
                                           double kappa_rate_fb,
                                           double speed_fb) {
  // If |speed| < 0.2m/s reset steer deque data.
  constexpr double kSpeedLimit = 0.2;
  if (std::fabs(speed_fb) < kSpeedLimit) {
    ResetSteer();
    return false;
  }

  kappa_cmd_cache_.push_back(kappa_cmd);

  kapparate_fb_cache_.push_back(kappa_rate_fb);
  const double mean_kapparate_fb =
      ComputerMeanOfCircularBuffer(kapparate_fb_cache_);

  kapparate_cmd_cache_.push_front(kappa_rate_cmd);
  boost::circular_buffer<double> kapparate_cmd_tmp;
  for (int i = kapparate_cmd_cache_.size() - num_steer_mean_;
       i < kapparate_cmd_cache_.size(); ++i) {
    kapparate_cmd_tmp.push_back(kapparate_cmd_cache_[i]);
  }
  const double mean_kapparate_cmd =
      ComputerMeanOfCircularBuffer(kapparate_cmd_cache_);

  const auto kappa_error = kappa_cmd_cache_.front() - kappa_fb;
  const auto kappa_rate_error = mean_kapparate_cmd - mean_kapparate_fb;

  // Debug
  debug_.set_mean_kapparate_cmd(mean_kapparate_cmd);
  debug_.set_mean_kapparate_fb(mean_kapparate_fb);
  debug_.set_kappa_error(kappa_error);
  debug_.set_kappa_rate_error(kappa_rate_error);
  debug_.set_kappa_fb(kappa_fb);
  debug_.set_kappa_cmd_delay(kappa_cmd_cache_.front());

  if (std::fabs(kappa_rate_error) >= config_.steer_kapparate_threshold ||
      std::fabs(kappa_error) >= config_.steer_kappa_threshold) {
    return true;
  }
  return false;
}

void WireControlChecker::ResetSteer() {
  kapparate_cmd_cache_.assign(num_steer_cmd_, 0.0);
  kapparate_fb_cache_.assign(num_steer_mean_, 0.0);
  kappa_cmd_cache_.assign(num_steer_delay_, 0.0);
}

WireControlCheckDebugProto WireControlChecker::GetCheckDebug() {
  return debug_;
}

}  // namespace control
}  // namespace qcraft
