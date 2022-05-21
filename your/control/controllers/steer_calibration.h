#ifndef ONBOARD_CONTROL_CONTROLLERS_STEER_GAIN_CALIBRATION_H_
#define ONBOARD_CONTROL_CONTROLLERS_STEER_GAIN_CALIBRATION_H_
#include <math.h>

#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/controller_util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

class SteerCalibration {
 public:
  SteerCalibration(const ControllerConf& control_conf, const double wheel_base);

  double SteerCalibrationMain(double steer_kappa, double steer_kappa_past,
                              double speed, double wheel_base,
                              double steer_ratio, double max_steer_angle,
                              SteerCalibrationDebugProto* debug);
  // steer_kappa_past is previous 0.1s steer_kappa

 private:
  double DeadzoneGain(double steer_kappa, double wheelbase) const;
  double AntiGapCompensate(double steer_kappa, double steer_kappa_past);

  bool enable_dynamic_model_compensation_ = false;
  double mf_ = 0.0;
  double mr_ = 0.0;
  double cf_ = 0.0;
  double cr_ = 0.0;
  double wheelbase_f = 0.0;
  double wheelbase_r = 0.0;
  double wheelbase_ = 0.0;
  double sliding_factor_ = 0.0;

  bool has_steering_gain_wrt_speed_ = false;
  PiecewiseLinearFunction<double> steering_gain_wrt_speed_plf_;

  bool enable_adapt_deadzone_ = false;
  double steer_straight_th_ = 0.0;
  double steer_turn_th_ = 0.0;
  double steer_straight_gain_ = 0.0;
  double steer_turn_gain_ = 0.0;
  double steer_gap_kappa_ = 0.0;
  double steer_gap_kappa_compensate_last_ = 0.0;
  double anti_gap_sign_ = 1.0;
};

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_
