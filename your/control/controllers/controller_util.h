#ifndef ONBOARD_CONTROL_CONTROLLER_CONTROLLER_UTIL_H_
#define ONBOARD_CONTROL_CONTROLLER_CONTROLLER_UTIL_H_

#include <vector>

#include "boost/circular_buffer.hpp"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::control {

bool IsCmdBackToZero(const std::vector<double>& cmd);

double Kappa2FrontWheelAngle(double kappa, double wheel_base);

double Kappa2SteerAngle(double kappa, double wheel_base, double steer_ratio,
                        double max_steer_angle = 0);

double Kappa2SteerPercentage(double kappa, double wheel_base,
                             double steer_ratio, double max_steer_angle);

double KappaRate2SteerRate(double kappa_rate, double kappa, double wheel_base,
                           double steer_ratio);

double FrontWheelAngle2Kappa(double front_wheel_angle, double wheel_base);

double SteerAngle2Kappa(double steer_angle, double wheel_base,
                        double steer_ratio);

double FrontWheelSteerRate2KappaRateWithKappa(double front_wheel_steer_rate,
                                              double kappa, double wheel_base);

double SteeringSpeed2KappaRateWithKappa(double steering_speed, double kappa,
                                        double wheel_base, double steer_ratio);

double FrontWheelSteerRate2KappaRateWithFrontWheelAngle(
    double front_wheel_steer_rate, double front_wheel_angle, double wheel_base);

double SteeringSpeed2KappaRateWithFrontWheelAngle(double steering_speed,
                                                  double front_wheel_angle,
                                                  double wheel_base,
                                                  double steer_ratio);

double SteeringPct2FrontWheelAngle(double steering_percentage,
                                   double steer_ratio, double max_steer_angle);

double ComputerMeanOfCircularBuffer(const boost::circular_buffer<double>& data);

double ClampKappaByMaxSteerAngle(double kappa, double wheel_base,
                                 double steer_ratio,
                                 double max_steer_angle = 0);

double ClampFntWhlAngleByMaxSteerAngle(double front_wheel_angle,
                                       double wheel_base, double steer_ratio,
                                       double max_steer_angle = 0);

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_CONTROLLER_CONTROLLER_UTIL_H_
