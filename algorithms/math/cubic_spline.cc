#include "onboard/math/cubic_spline.h"

#include "onboard/math/eigen.h"
#include "onboard/math/util.h"

namespace qcraft {

void CubicSpline::Solve() {
  const int n = x_.size();
  SMatXd A(n, n);
  VecXd b(n, 1);
  constexpr double three_inv = 1.0 / 3.0;
  for (int i = 1; i < n - 1; i++) {
    A.insert(i, i - 1) = 1.0 * three_inv * (x_[i] - x_[i - 1]);
    A.insert(i, i) = 2.0 * three_inv * (x_[i + 1] - x_[i - 1]);
    A.insert(i, i + 1) = 1.0 * three_inv * (x_[i + 1] - x_[i]);
    b(i) = (y_[i + 1] - y_[i]) / (x_[i + 1] - x_[i]) -
           (y_[i] - y_[i - 1]) / (x_[i] - x_[i - 1]);
  }
  // Set boundary conditions.
  if (left_.type == BoundaryType::SOD) {
    A.insert(0, 0) = 2.0;
    A.insert(0, 1) = 0.0;
    b(0) = left_.value;
  } else if (left_.type == BoundaryType::FOD) {
    A.insert(0, 0) = 2.0 * (x_[1] - x_[0]);
    A.insert(0, 1) = 1.0 * (x_[1] - x_[0]);
    b(0) = 3.0 * ((y_[1] - y_[0]) / (x_[1] - x_[0]) - left_.value);
  } else {
    QLOG(FATAL) << "Unknown left boundary type "
                << static_cast<int>(left_.type);
  }
  if (right_.type == BoundaryType::SOD) {
    A.insert(n - 1, n - 1) = 2.0;
    A.insert(n - 1, n - 2) = 0.0;
    b(n - 1) = right_.value;
  } else if (right_.type == BoundaryType::FOD) {
    A.insert(n - 1, n - 1) = 2.0 * (x_[n - 1] - x_[n - 2]);
    A.insert(n - 1, n - 2) = 1.0 * (x_[n - 1] - x_[n - 2]);
    b(n - 1) = 3.0 * (right_.value -
                      (y_[n - 1] - y_[n - 2]) / (x_[n - 1] - x_[n - 2]));
  } else {
    QLOG(FATAL) << "Unknown right boundary type "
                << static_cast<int>(right_.type);
  }

  A.makeCompressed();
  Eigen::SparseLU<SMatXd> solver;
  solver.analyzePattern(A);
  solver.factorize(A);
  QCHECK(solver.info() == Eigen::Success);

  const auto c = solver.solve(b);
  c_.clear();
  c_.reserve(n);
  for (int i = 0; i < n; ++i) {
    c_.push_back(c(i));
  }

  d_.clear();
  d_.resize(n);
  b_.clear();
  b_.resize(n);
  for (int i = 0; i < n - 1; ++i) {
    d_[i] = 1.0 * three_inv * (c_[i + 1] - c_[i]) / (x_[i + 1] - x_[i]);
    b_[i] = (y_[i + 1] - y_[i]) / (x_[i + 1] - x_[i]) -
            1.0 * three_inv * (2.0 * c_[i] + c_[i + 1]) * (x_[i + 1] - x_[i]);
  }
  const double h = x_[n - 1] - x_[n - 2];
  d_[n - 1] = 0.0;
  b_[n - 1] = 3.0 * d_[n - 2] * Sqr(h) + 2.0 * c_[n - 2] * h + b_[n - 2];
  if (right_.type == BoundaryType::FOD) {
    c_[n - 1] = 0.0;
  }

  c0_ = (left_.type == BoundaryType::FOD) ? 0.0 : c_[0];
}

double CubicSpline::Evaluate(double x) const {
  const int n = x_.size();
  const int idx = FindNearestIndex(x);
  const double h = x - x_[idx];

  double value;
  if (x < x_[0]) {
    // Extrapolation to the left.
    value = (c0_ * h + b_[0]) * h + y_[0];
  } else if (x > x_[n - 1]) {
    // Extrapolation to the right
    value = (c_[n - 1] * h + b_[n - 1]) * h + y_[n - 1];
  } else {
    // Interpolation.
    value = ((d_[idx] * h + c_[idx]) * h + b_[idx]) * h + y_[idx];
  }

  return value;
}

double CubicSpline::EvaluateDerivative(int order, double x) const {
  QCHECK_GT(order, 0);
  const int n = x_.size();
  const int idx = FindNearestIndex(x);
  const double h = x - x_[idx];

  double value;
  if (x < x_[0]) {
    switch (order) {
      case 1:
        value = 2.0 * c0_ * h + b_[0];
        break;
      case 2:
        value = 2.0 * c0_;
        break;
      default:
        value = 0.0;
        break;
    }
  } else if (x > x_[n - 1]) {
    switch (order) {
      case 1:
        value = 2.0 * c_[n - 1] * h + b_[n - 1];
        break;
      case 2:
        value = 2.0 * c_[n - 1];
        break;
      default:
        value = 0.0;
        break;
    }
  } else {
    switch (order) {
      case 1:
        value = (3.0 * d_[idx] * h + 2.0 * c_[idx]) * h + b_[idx];
        break;
      case 2:
        value = 6.0 * d_[idx] * h + 2.0 * c_[idx];
        break;
      case 3:
        value = 6.0 * d_[idx];
        break;
      default:
        value = 0.0;
        break;
    }
  }

  return value;
}

}  // namespace qcraft
