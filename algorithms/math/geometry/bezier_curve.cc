#include "onboard/math/geometry/bezier_curve.h"

#include <algorithm>
#include <cmath>

#include "Eigen/LU"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace Bezier {
namespace {

template <int N = 64>
class BinomialCoeffTable {
 public:
  BinomialCoeffTable() {
    for (int i = 0; i < N; ++i) {
      coeff_table_[i][0] = 1;
      for (int j = 1; j < i; ++j) {
        coeff_table_[i][j] =
            coeff_table_[i - 1][j - 1] + coeff_table_[i - 1][j];
      }
      coeff_table_[i][i] = 1;
    }
  }

  int64_t operator()(int i, int j) const {
    QCHECK_LT(i, N);
    QCHECK_LE(j, i);
    return coeff_table_[i][j];
  }

 private:
  int64_t coeff_table_[N][N] = {{0}};
};

int64_t BinomialCoeff(int i, int j) {
  static BinomialCoeffTable binomial_coeff_table;
  return binomial_coeff_table(i, j);
}

}  // namespace

std::vector<int64_t> GetBinomialCoeff(int n) {
  QCHECK_GE(n, 1);

  std::vector<int64_t> coeffs(n + 1);
  for (int i = 0; i < n + 1; ++i) {
    coeffs[i] = BinomialCoeff(n, i);
  }
  return coeffs;
}

Vec2d SampleBezier(absl::Span<const Vec2d> control_points, double t) {
  QCHECK_GT(control_points.size(), 1);
  QCHECK_GE(t, 0.0);
  QCHECK_LE(t, 1.0);

  if (t == 1.0) {
    return control_points.back();
  }
  const int n = control_points.size() - 1;
  const auto binomial_coeffs = GetBinomialCoeff(n);
  Vec2d point = Vec2d::Zero();
  double x = 1.0;
  double y = std::pow(1.0 - t, n);
  const double one_minus_t_inv = 1.0 / (1.0 - t);
  for (int i = 0; i < n + 1; ++i) {
    point += binomial_coeffs[i] * x * y * control_points[i];
    x *= t;
    y *= one_minus_t_inv;
  }
  return point;
}

Vec2d SampleBezierWithBinomialCoeffs(absl::Span<const Vec2d> control_points,
                                     absl::Span<const int64_t> binomial_coeffs,
                                     double t) {
  QCHECK_GT(control_points.size(), 1);
  QCHECK_EQ(control_points.size(), binomial_coeffs.size());
  QCHECK_GE(t, 0.0);
  QCHECK_LE(t, 1.0);

  if (t == 1.0) {
    return control_points.back();
  }
  const int n = control_points.size() - 1;
  Vec2d point = Vec2d::Zero();
  double x = 1.0;
  double y = std::pow(1.0 - t, n);
  const double one_minus_t_inv = 1.0 / (1.0 - t);
  for (int i = 0; i < n + 1; ++i) {
    point += binomial_coeffs[i] * x * y * control_points[i];
    x *= t;
    y *= one_minus_t_inv;
  }
  return point;
}

Vec2d SampleBezierTangent(absl::Span<const Vec2d> control_points, double t) {
  QCHECK_GT(control_points.size(), 1);
  QCHECK_GE(t, 0.0);
  QCHECK_LE(t, 1.0);

  if (t == 1.0) {
    return (control_points.back() - control_points[control_points.size() - 2])
        .normalized();
  }
  const int n = control_points.size() - 1;
  const auto binomial_coeffs = GetBinomialCoeff(n - 1);
  Vec2d tangent = Vec2d::Zero();
  double x = 1.0;
  double y = std::pow(1.0 - t, n - 1);
  const double one_minus_t_inv = 1.0 / (1.0 - t);
  for (int i = 0; i < n; ++i) {
    tangent += n * binomial_coeffs[i] * x * y *
               (control_points[i + 1] - control_points[i]);
    x *= t;
    y *= one_minus_t_inv;
  }
  return tangent.normalized();
}

Vec2d SampleBezierTangentWithBinomialCoeffs(
    absl::Span<const Vec2d> control_points,
    absl::Span<const int64_t> binomial_coeffs, double t) {
  QCHECK_GT(control_points.size(), 1);
  QCHECK_EQ(control_points.size(), binomial_coeffs.size() + 1);
  QCHECK_GE(t, 0.0);
  QCHECK_LE(t, 1.0);

  if (t == 1.0) {
    return (control_points.back() - control_points[control_points.size() - 2])
        .normalized();
  }
  const int n = control_points.size() - 1;
  Vec2d tangent = Vec2d::Zero();
  double x = 1.0;
  double y = std::pow(1.0 - t, n - 1);
  const double one_minus_t_inv = 1.0 / (1.0 - t);
  for (int i = 0; i < n; ++i) {
    tangent += n * binomial_coeffs[i] * x * y *
               (control_points[i + 1] - control_points[i]);
    x *= t;
    y *= one_minus_t_inv;
  }
  return tangent.normalized();
}

Vec2d SampleThirdOrderBezier(absl::Span<const Vec2d> control_points, double t) {
  QCHECK_EQ(control_points.size(), 4);
  QCHECK_GE(t, 0.0);
  QCHECK_LE(t, 1.0);

  return Cube(1.0 - t) * control_points[0] +
         3.0 * t * Sqr(1.0 - t) * control_points[1] +
         3.0 * Sqr(t) * (1.0 - t) * control_points[2] +
         Cube(t) * control_points[3];
}

Vec2d SampleThirdOrderBezierTangent(absl::Span<const Vec2d> control_points,
                                    double t) {
  QCHECK_EQ(control_points.size(), 4);
  QCHECK_GE(t, 0.0);
  QCHECK_LE(t, 1.0);

  return (3.0 * Sqr(1.0 - t) * (control_points[1] - control_points[0]) +
          6.0 * t * (1.0 - t) * (control_points[2] - control_points[1]) +
          3.0 * Sqr(t) * (control_points[3] - control_points[2]))
      .normalized();
}

std::pair<std::vector<Vec2d>, std::vector<Vec2d>> SplitBezier(
    absl::Span<const Vec2d> control_points, double t) {
  const int control_point_size = control_points.size();
  const int order = control_points.size() - 1;
  std::vector<Vec2d> left(control_point_size, control_points.front());
  std::vector<Vec2d> right(control_point_size, control_points.back());

  std::vector<Vec2d> prev(control_points.begin(), control_points.end());
  std::vector<Vec2d> curr(control_points.begin(), control_points.end());

  // de Casteljau: https://pomax.github.io/bezierinfo/#splitting
  int subs = 0;
  while (subs < order) {
    for (int i = 0; i < order - subs; i++) {
      curr[i].x() = (1.0f - t) * prev[i].x() + t * prev[i + 1].x();
      curr[i].y() = (1.0f - t) * prev[i].y() + t * prev[i + 1].y();
      if (i == 0) left[subs + 1] = curr[i];
      if (i == (order - subs - 1)) right[subs + 1] = curr[i];
    }
    std::swap(prev, curr);
    subs++;
  }

  return {left, right};
}

absl::Status FitBezier(absl::Span<const Vec2d> polyline_points,
                       std::vector<Vec2d> *control_points) {
  QCHECK(control_points);
  const int n = polyline_points.size();
  QCHECK_GE(n, 2);
  // Create and assign matrix M.
  Eigen::MatrixXd M;
  M.resize(n, n);
  M.setZero();
  for (int i = 0; i < n; ++i) {
    int sign = (i & 1) ? -1 : 1;
    for (int j = 0; j < i + 1; ++j) {
      M(i, j) =
          BinomialCoeff(n - 1, j) * BinomialCoeff(n - 1 - j, i - j) * sign;
      sign *= -1;
    }
  }
  // Create and assign matrix T.
  Eigen::MatrixXd T;
  T.resize(n, n);
  const double delta_t = 1.0 / (n - 1);
  double current_t = 0;
  for (int i = 0; i < n; ++i) {
    double current_ex_t = 1;
    for (int j = 0; j < n; ++j) {
      T(i, j) = current_ex_t;
      current_ex_t *= current_t;
    }
    current_t += delta_t;
  }
  // Create and assign matrix P.
  Eigen::MatrixXd P;
  P.resize(n, 2);
  for (int i = 0; i < n; ++i) {
    P(i, 0) = polyline_points[i].x();
    P(i, 1) = polyline_points[i].y();
  }
  // Create and assign matrix C.
  Eigen::MatrixXd C;
  C.resize(n, 2);
  C = M.inverse() * (T.transpose() * T).inverse() * T.transpose() * P;
  // Assignment output vector control_points.
  control_points->clear();
  control_points->reserve(n);
  for (int i = 0; i < n; ++i) {
    control_points->emplace_back(C(i, 0), C(i, 1));
  }
  constexpr int kMaxFitPointNum = 10;
  if (n > kMaxFitPointNum) {
    LOG(ERROR) << "points num is more";
    return absl::OutOfRangeError(
        absl::StrCat("If the number of data points is greater than ",
                     kMaxFitPointNum, ", the fitting may be problematic."));
  }
  return absl::OkStatus();
}

}  // namespace Bezier
}  // namespace qcraft
