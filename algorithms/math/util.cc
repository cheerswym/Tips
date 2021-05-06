#include "onboard/math/util.h"

namespace qcraft {

std::vector<double> QuadraticRoot(double a, double b, double c) {
  constexpr double kEpsilon = 1e-10;
  if (std::abs(a) < kEpsilon) {
    if (std::abs(b) < kEpsilon) return {};
    return {-c / b};
  }
  const double d = b * b - 4 * a * c;
  if (d < 0.0) return {};
  const double sqrt_d = std::sqrt(d);
  const double t = 0.5 / a;
  std::vector<double> roots = {t * (-b + sqrt_d), t * (-b - sqrt_d)};
  if (roots[0] > roots[1]) std::swap(roots[0], roots[1]);
  return roots;
}

}  // namespace qcraft
