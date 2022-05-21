#ifndef ONBOARD_CONTROL_ANTI_WINDUP_INTEGRATOR_H_
#define ONBOARD_CONTROL_ANTI_WINDUP_INTEGRATOR_H_

#include "onboard/control/proto/controller_conf.pb.h"

namespace qcraft {
namespace control {
class AntiWindupIntegrator {
 public:
  // Read controller config and integrator type
  void Init(const IntegratorConf& integrator_conf, double ts);

  // Set last_integral_value_ and last_integrand_value_ to zero.
  void Clear();

  // Call clear() if enabled_ == false or is_standstill == true, elsewise
  // integrate with anti-windup schemes.
  void Integrate(double integrand_value, bool is_standstill);

  // Return the integral value.
  double GetIntegralValue() const;

  // Return the integral alpha value (used in mpc state matrix).
  double GetIntegralAlphaValue() const;

 private:
  double ts_ = 0.2;
  double integral_alpha_ = 0.0;
  double integral_lower_bound_ = 0.0;
  double integral_upper_bound_ = 0.0;
  double difference_lower_bound_ = 0.0;
  double difference_upper_bound_ = 0.0;

  double last_integral_value_ = 0.0;
  double last_integrand_value_ = 0.0;
};
}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_ANTI_WINDUP_INTEGRATOR_H_
