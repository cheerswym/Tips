#ifndef ONBOARD_MATH_POLYNOMIAL_FITTER_H_
#define ONBOARD_MATH_POLYNOMIAL_FITTER_H_

#include <limits>
#include <utility>
#include <vector>

#include "onboard/math/vec.h"
#include "onboard/utils/string_util.h"

namespace qcraft {

/*
  Solver for linear least squares systems (LS).
   - SVD: The SVD decomposition, generally the most accurate yet the slowest.
   - QR: The QR decomposition, moderately accurate and fast.
   - PINV: The pseudo-inverse, the fastest but least accurate.
 */
enum LS_SOLVER {
  SVD = 0,
  QR = 1,
  PINV = 2,
};

/*
  Fit a n-degree polynomial from input training data such that
    f(x) = beta_0 + beta_1 * x + beta_2 * x^2 + ... + beta_n * x^n
  via the weighted least squares (LS) methed.

  See
    https://en.wikipedia.org/wiki/Polynomial_regression
  and
    https://en.wikipedia.org/wiki/Weighted_least_squares
  for more detail.
 */

class PolynomialFitter {
 public:
  PolynomialFitter()
      : degree_(0),
        mse_(std::numeric_limits<double>::infinity()),
        debug_(false) {}
  PolynomialFitter(int degree, std::vector<Vec2d> data, bool debug = false)
      : degree_(degree),
        data_(std::move(data)),
        mse_(std::numeric_limits<double>::infinity()),
        debug_(debug) {
    CheckData();
  }
  PolynomialFitter(int degree, std::vector<Vec2d> data,
                   std::vector<double> weights, bool debug = false,
                   bool check_data_size = true)
      : degree_(degree),
        data_(std::move(data)),
        weights_(std::move(weights)),
        mse_(std::numeric_limits<double>::infinity()),
        debug_(debug) {
    CheckData(check_data_size);
  }

  void CheckData(bool check_data_size = true) const;
  void LoadData(const std::vector<Vec2d> data,
                const std::vector<double> weights = std::vector<double>(),
                bool check_data_size = true) {
    data_ = std::move(data);
    weights_ = std::move(weights);
    CheckData(check_data_size);
  }
  void SetDegree(int d) { degree_ = d; }
  void FitData(LS_SOLVER solver = LS_SOLVER::SVD, bool compute_mse = false);
  double Evaluate(double x) const;
  double FitPointError(const Vec2d point) const;
  const std::vector<double> &beta() const { return beta_; }
  std::vector<double> &mutable_beta() { return beta_; }
  const double mse() const { return mse_; }

  void PrintDebugInfo() const;

 private:
  int degree_;
  std::vector<qcraft::Vec2d> data_;  // x, y
  std::vector<double> weights_;
  std::vector<double> beta_;
  double mse_;
  bool debug_;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_POLYNOMIAL_FITTER_H_
