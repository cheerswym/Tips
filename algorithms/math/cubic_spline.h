
#ifndef ONBOARD_MATH_CUBIC_SPLINE_H_
#define ONBOARD_MATH_CUBIC_SPLINE_H_

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"

namespace qcraft {

class CubicSpline {
 public:
  enum class BoundaryType {
    FOD = 0,  // First-order derivative.
    SOD = 1   // Second-order derivative.
  };
  struct BoundaryCondition {
    BoundaryType type = BoundaryType::SOD;
    double value = 0.0;
  };

  CubicSpline(std::vector<double> x, std::vector<double> y)
      : x_(std::move(x)),
        y_(std::move(y)),
        left_({.type = BoundaryType::SOD, .value = 0.0}),
        right_({.type = BoundaryType::SOD, .value = 0.0}) {
    CheckInput(x_, y_);
    Solve();
  }

  CubicSpline(std::vector<double> x, std::vector<double> y,
              BoundaryCondition left, BoundaryCondition right)
      : x_(std::move(x)), y_(std::move(y)), left_(left), right_(right) {
    CheckInput(x_, y_);
    Solve();
  }

  // Evaluate value and derivatives at a given x.
  double Evaluate(double x) const;
  double EvaluateDerivative(int order, double x) const;

  const std::vector<double>& x() const { return x_; }
  const std::vector<double>& y() const { return y_; }

 protected:
  int FindNearestIndex(double x) const {
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    const int idx = std::max(static_cast<int>(it - x_.begin() - 1), 0);
    return idx;
  }

  void CheckInput(const std::vector<double>& x, const std::vector<double>& y) {
    QCHECK_EQ(x.size(), y.size());
    QCHECK_GT(x.size(), 1);
    // Check strict monotonicity of x.
    for (int i = 0; i < x.size() - 1; ++i) {
      QCHECK_LT(x[i], x[i + 1]);
    }
  }

  void Solve();

  std::vector<double> x_;  // x coordinates.
  std::vector<double> y_;  // y coordinates.

  // f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3 where a_i = y_i
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;

  double c0_;
  BoundaryCondition left_;
  BoundaryCondition right_;
};
}  // namespace qcraft

#endif  // ONBOARD_MATH_CUBIC_SPLINE_H_
