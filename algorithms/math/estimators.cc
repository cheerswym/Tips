#include "onboard/math/estimators.h"

namespace qcraft {

Vec2d QuadraticFit2D(const Mat3d &patch, double dx, double dy, Mat2d *A,
                     double *c) {
  // Parameters to solve for: a00, a01, a11, rx, ry, c
  // f(x) = (x - r)^T A (x - r) + c
  // f(x, y) = {x - rx \\ y - ry} {a00 & a01 \\ a01 & a11} {x - rx \\ y - ry}^T
  //           + c
  //         = a00 (x - rx)^2 + 2 a01 (x - rx) (y - ry) + a11 (y - ry)^2 + c
  //         = a00 x^2 + 2a01 xy + a11 y^2
  //           - (2a00 rx + 2a01 ry) x - (2a11 ry + 2a01 rx) y
  //           + a00 rx^2 + 2a01 rx ry + a11 ry^2 + c
  //
  // f(x, y) = g1 x^2 + g2 xy + g3 y^2 + g4 x + g5 y + g6
  //      g1 = a00
  //      g2 = 2a01
  //      g3 = a11
  //      g4 = -(2a00 rx + 2a01 ry)
  //      g5 = -(2a11 ry + 2a01 rx)
  //      g6 = a00 rx^2 + 2a01 rx ry + a11 ry^2 + c
  //
  MatXd B = MatXd::Zero(9, 6);
  VecXd rhs = VecXd::Zero(9);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      const double x = dx * (col - 1);
      const double y = dy * (row - 1);
      const int i = row * 3 + col;
      B(i, 0) = x * x;
      B(i, 1) = x * y;
      B(i, 2) = y * y;
      B(i, 3) = x;
      B(i, 4) = y;
      B(i, 5) = 1;
      rhs(i) = patch(row, col);
    }
  }

  // Solve the over-constrained system through normal equation.
  const VecXd coeffs =
      (B.transpose() * B).partialPivLu().solve(B.transpose() * rhs);
  const double g1 = coeffs[0];  // a00
  const double g2 = coeffs[1];  // 2a01
  const double g3 = coeffs[2];  // a11
  const double g4 = coeffs[3];
  const double g5 = coeffs[4];
  const double g6 = coeffs[5];
  if (A != nullptr) {
    (*A)(0, 0) = g1;
    (*A)(0, 1) = (*A)(1, 0) = g2 * 0.5;
    (*A)(1, 1) = g3;
  }

  // Solve rx and ry from:
  // 2a00 rx + 2a01 ry = -g4
  // 2a01 rx + 2a11 ry = -g5
  Vec2d r = Vec2d::Zero();
  const double denom = g2 * g2 - 4 * g1 * g3;
  if (std::abs(denom) < 1e-10) {
    // Matrix A is rank-deficient: the extremum is a 1D line so a single
    // extremum point is undefined. Just return a particular solution with
    // regularization that rx == ry.
    r.x() = -g4 / (2 * g1 + g2);
    r.y() = r.x();
  } else {
    // Matrix A is well-conditioned. Solve for rx and ry.
    r.x() = (g4 * 2 * g3 - g5 * g2) / denom;
    r.y() = (g5 * 2 * g1 - g4 * g2) / denom;
  }
  if (c != nullptr) {
    *c = g6 - g1 * r.x() * r.x() - g2 * r.x() * r.y() - g3 * r.y() * r.y();
  }

  return r;
}

}  // namespace qcraft
