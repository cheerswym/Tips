#include "onboard/control/controllers/controller_util.h"

#include <math.h>

#include "onboard/math/util.h"

namespace qcraft::control {

bool IsCmdBackToZero(const std::vector<double>& cmd) {
  for (int i = 0; i + 1 < cmd.size(); ++i) {
    const double cmd_current = cmd[i];
    const double cmd_next = cmd[i + 1];
    const double delta_cmd = cmd_next - cmd_current;

    if (cmd_current * delta_cmd > 0 || cmd_current * cmd_next < 0) {
      return false;
    }
  }

  return true;
}

double Kappa2FrontWheelAngle(double kappa, double wheel_base) {
  return std::atan(kappa * wheel_base);
}

double FrontWheelAngle2Kappa(double front_wheel_angle, double wheel_base) {
  return std::copysign(std::tan(front_wheel_angle) / wheel_base,
                       front_wheel_angle);
}

double Kappa2SteerAngle(double kappa, double wheel_base, double steer_ratio,
                        double max_steer_angle) {
  const double front_wheel_angle = Kappa2FrontWheelAngle(kappa, wheel_base);
  const double steer_angle = front_wheel_angle * steer_ratio;
  if (max_steer_angle > 0.0) {
    return std::clamp(steer_angle, -max_steer_angle, max_steer_angle);
  } else {  // not set
    return steer_angle;
  }
}

double Kappa2SteerPercentage(double kappa, double wheel_base,
                             double steer_ratio, double max_steer_angle) {
  const double steer_angle = Kappa2SteerAngle(kappa, wheel_base, steer_ratio);
  return std::clamp(steer_angle / max_steer_angle, -1.0, 1.0) * 100.0;
}

double KappaRate2SteerRate(double kappa_rate, double kappa, double wheel_base,
                           double steer_ratio) {
  // tan(delta) = l * kappa;
  // d(delta)/dt = l * cos^2(delta) d(kappa) /dt;
  const double front_wheel_angle = Kappa2FrontWheelAngle(kappa, wheel_base);
  const double front_wheel_angle_rate =
      wheel_base * Sqr(std::cos(front_wheel_angle)) * kappa_rate;
  return front_wheel_angle_rate * steer_ratio;
}

double SteerAngle2Kappa(double steer_angle, double wheel_base,
                        double steer_ratio) {
  const double front_wheel_angle = steer_angle / steer_ratio;
  return FrontWheelAngle2Kappa(front_wheel_angle, wheel_base);
}

double FrontWheelSteerRate2KappaRateWithKappa(double front_wheel_steer_rate,
                                              double kappa, double wheel_base) {
  // d(kappa)/dt = d(delta)/dt /l / cos^2(delta);
  const double front_wheel_steer = Kappa2FrontWheelAngle(kappa, wheel_base);
  return front_wheel_steer_rate / wheel_base / Sqr(std::cos(front_wheel_steer));
}

double SteeringSpeed2KappaRateWithKappa(double steering_speed, double kappa,
                                        double wheel_base, double steer_ratio) {
  // d(kappa)/dt = d(delta)/dt /l / cos^2(delta);
  const double front_wheel_steer = Kappa2FrontWheelAngle(kappa, wheel_base);
  const double front_wheel_steer_rate = steering_speed / steer_ratio;
  return front_wheel_steer_rate / wheel_base / Sqr(std::cos(front_wheel_steer));
}

double FrontWheelSteerRate2KappaRateWithFrontWheelAngle(
    double front_wheel_steer_rate, double front_wheel_angle,
    double wheel_base) {
  // d(kappa)/dt = d(delta)/dt /l / cos^2(delta);
  return front_wheel_steer_rate / wheel_base / Sqr(std::cos(front_wheel_angle));
}

double SteeringSpeed2KappaRateWithFrontWheelAngle(double steering_speed,
                                                  double front_wheel_angle,
                                                  double wheel_base,
                                                  double steer_ratio) {
  // d(kappa)/dt = d(delta)/dt /l / cos^2(delta);
  const double front_wheel_steer_rate = steering_speed / steer_ratio;
  return front_wheel_steer_rate / wheel_base / Sqr(std::cos(front_wheel_angle));
}

double SteeringPct2FrontWheelAngle(double steering_percentage,
                                   double steer_ratio, double max_steer_angle) {
  return steering_percentage * 0.01 * max_steer_angle / steer_ratio;
}

double ComputerMeanOfCircularBuffer(
    const boost::circular_buffer<double>& data) {
  if (data.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (int i = 0; i < data.size(); ++i) {
    sum += data[i];
  }
  return sum / data.size();
}

double ClampKappaByMaxSteerAngle(double kappa, double wheel_base,
                                 double steer_ratio, double max_steer_angle) {
  if (max_steer_angle > 0.0) {
    const double max_kappa =
        SteerAngle2Kappa(max_steer_angle, wheel_base, steer_ratio);
    return std::clamp(kappa, -max_kappa, max_kappa);
  }
  return kappa;
}

double ClampFntWhlAngleByMaxSteerAngle(double front_wheel_angle,
                                       double wheel_base, double steer_ratio,
                                       double max_steer_angle) {
  if (max_steer_angle > 0.0) {
    const double max_kappa =
        SteerAngle2Kappa(max_steer_angle, wheel_base, steer_ratio);
    const double max_front_wheel_angle =
        Kappa2FrontWheelAngle(max_kappa, wheel_base);
    return std::clamp(front_wheel_angle, -max_front_wheel_angle,
                      max_front_wheel_angle);
  }
  return front_wheel_angle;
}

}  // namespace qcraft::control
