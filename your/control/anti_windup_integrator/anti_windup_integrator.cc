#include "onboard/control/anti_windup_integrator/anti_windup_integrator.h"

#include <algorithm>

#include "onboard/lite/logging.h"

namespace qcraft {
namespace control {
void AntiWindupIntegrator::Init(const IntegratorConf& integrator_conf,
                                double ts) {
  QCHECK(integrator_conf.has_integral_timer() &&
         integrator_conf.integral_timer() >= ts)
      << "integral_alpha does not exist or is less than ts = " << ts;
  QCHECK(integrator_conf.has_integral_bound() &&
         integrator_conf.integral_bound() > 0)
      << "integral_bound does not exist or is non-positive";
  QCHECK(integrator_conf.has_difference_bound() &&
         integrator_conf.difference_bound() > 0)
      << "difference_bound does not exist or is non-positive";
  QCHECK(ts > 0) << "Non-positive ts = " << ts;
  //
  // NOTE(yunchang): using the lpf time constant
  //                   T = -1 /log(alpha)
  //                   T = -ts/log(alpha_norm)
  // one gets
  //                   alpha      = exp(-1 /T)
  //                   alpha_norm = exp(-ts/T)
  // when desired T is several secs or larger, this might be linearized as
  //                   alpha      ~= 1 - 1  / T
  //                   alpha_norm ~= 1 - ts / T
  //
  // *: AntiWindupIntegrator::integral_alpha_, which is returned by
  //    GetIntegralAlphaValue(), is directly used in mpc controller.
  //
  ts_ = ts;
  integral_alpha_ = 1.0 - ts / integrator_conf.integral_timer();
  integral_lower_bound_ = integrator_conf.integral_bound() * -1.0;
  integral_upper_bound_ = integrator_conf.integral_bound();
  difference_lower_bound_ = ts * integrator_conf.difference_bound() * -1.0;
  difference_upper_bound_ = ts * integrator_conf.difference_bound();
}

void AntiWindupIntegrator::Clear() {
  last_integral_value_ = 0.0;
  last_integrand_value_ = 0.0;
}

void AntiWindupIntegrator::Integrate(double integrand_value,
                                     bool is_standstill) {
  const double new_integrand_value_ = std::clamp(
      integrand_value, last_integrand_value_ + difference_lower_bound_,
      last_integrand_value_ + difference_upper_bound_);
  if (is_standstill || new_integrand_value_ * last_integrand_value_ < 0) {
    Clear();
  } else {
    last_integrand_value_ = new_integrand_value_;
    last_integral_value_ = std::clamp(
        last_integral_value_ * integral_alpha_ + last_integrand_value_ * ts_,
        integral_lower_bound_, integral_upper_bound_);
  }
}

double AntiWindupIntegrator::GetIntegralValue() const {
  return last_integral_value_;
}

double AntiWindupIntegrator::GetIntegralAlphaValue() const {
  return integral_alpha_;
}

}  // namespace control
}  // namespace qcraft
