#ifndef ONBOARD_CONTROL_CONTROL_CHECK_WIRE_CONTROL_CHECK_H_
#define ONBOARD_CONTROL_CONTROL_CHECK_WIRE_CONTROL_CHECK_H_

#include <memory>

#include "boost/circular_buffer.hpp"
#include "onboard/control/proto/controller_conf.pb.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

struct WireControlCheckConfig {
  double control_period = 0.01;  // control period from control config param
  double steer_delay = 0.5;      // steer_delay_time from control config param

  // Check hard throttle
  double throttle_fb_time = 0.1;     // continuous check time when hard throttle
  double throttle_delay_low = 0.3;   // throttle_delay_low_time
  double throttle_delay_high = 1.5;  // throttle_delay_high_time
  double acc_hard_threshold = 2.5;   // max feeback acceleration threshold
  double acc_cmd_threshold = 1.75;   // max cmd acceleration threshold

  // Check steer protect
  double steer_mean_time = 0.4;             // mean time of kappa rate pose
  double steer_kappa_threshold = 0.05;      // kappa error protect threshold
  double steer_kapparate_threshold = 0.02;  // kappa rate error threshold
  int window_mean_filter = 5;               // kappa mean filter window
};

class WireControlChecker {
 public:
  /**
   * @description: Initialize wire control check API.
   * @param config {Wire Control Check Config, include vehicle and control
   * check config}
   */
  WireControlChecker(double control_period, double steer_delay);

  ~WireControlChecker();

  bool CheckProc(AutonomyStateProto_State autonomy_state,
                 double acceleration_cmd, double acceleration_fb,
                 double kappa_cmd, double kappa_fb, double speed_fb,
                 Chassis::GearPosition gear_fb);

  WireControlCheckDebugProto GetCheckDebug();

 private:
  /**
   * @description: Check throttle status, if or not hard throttle. when
   * acceleration feedback continuous is >= 2.5m/s2, and has that acceleration
   * cmd all is < 1.75m/s2 in scope of 0.3s~1.5s.
   * @param acceleration_cmd {acceleration_cmd, from mpc controller}
   * @param acceleration_fb {acceleration_feedback, from chassis}
   * @param gear_fb {gear_feedback, from chassis}
   * @return {status: if or not hard throttle}
   */
  bool DoesAccelerateTooHard(double acceleration_cmd, double acceleration_fb);

  /**
   * @description: Check steer status, if or not big error between kappa rate
   * cmd and kappa rate feedback, by [error mean is >= protect_threshold]
   * @param kappa_rate_cmd {kappa_rate_cmd, from mpc controller}
   * @param kappa_rate_fb {kappa_rate_fb, from chassis}
   * @param gear_fb {gear_feedback, from chassis}
   * @param speed_fb {speed_feedback, from chassis}
   * @return {status: if or not active steer protect}
   */
  bool IsSteeringFailure(double kappa_cmd, double kappa_fb,
                         double kappa_rate_cmd, double kappa_rate_fb,
                         double speed_fb);

  WireControlCheckConfig config_;
  WireControlCheckDebugProto debug_;
  Chassis::GearPosition gear_fb_last_ = Chassis::GEAR_NONE;
  bool first_run_ = true;

  // Change gear, and reset throttle and steer cache data.
  bool UpdateGear(Chassis::GearPosition gear_fb);

  // Check hard throttle protect
  boost::circular_buffer<double> acceleration_fb_cache_;
  boost::circular_buffer<double> acceleration_cmd_cache_;
  int num_throttle_fb_ = 0;  // continuous cache num of hard throttle check time
  int num_throttle_cmd_ = 0;  // continuous cache num of throttle delay
  void ResetThrottle();

  // Check steer protect
  boost::circular_buffer<double> kapparate_fb_cache_;
  boost::circular_buffer<double> kapparate_cmd_cache_;
  boost::circular_buffer<double> kappa_cmd_cache_;
  std::unique_ptr<apollo::common::MeanFilter> kappa_cmd_filter_;
  std::unique_ptr<apollo::common::MeanFilter> kappa_fb_filter_;
  double kappa_fb_smooth_last_ = 0.0;
  double kappa_cmd_smooth_last_ = 0.0;
  int num_steer_mean_ = 0;   // continuous cache num of computer mean kapparate
  int num_steer_cmd_ = 0;    // continuous cache num of kapparate cmd check time
  int num_steer_delay_ = 0;  // continuous cache num of kapparate cmd check time
  void ResetSteer();
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_CHECK_WIRE_CONTROL_CHECK_H_
