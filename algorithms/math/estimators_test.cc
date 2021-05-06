#include "onboard/math/estimators.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kEps = 1e-10;

void ExpectA(const Mat2d &A, double a00, double a01, double a11) {
  EXPECT_NEAR(A(0, 0), a00, kEps);
  EXPECT_NEAR(A(0, 1), a01, kEps);
  EXPECT_NEAR(A(1, 0), a01, kEps);  // A is expected to be symmetric.
  EXPECT_NEAR(A(1, 1), a11, kEps);
}

TEST(EstimatorTest, Explicit_xy) {
  Mat3d patch;
  Vec2d r;
  Mat2d A;
  double c;

  // Simple local minimum.
  patch << 2, 1, 2, 1, 0, 1, 2, 1, 2;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Center shifted in y.
  patch << 5, 4, 5, 2, 1, 2, 1, 0, 1;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 1.0, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Center shifted in x and y.
  patch << 8, 5, 4, 5, 2, 1, 4, 1, 0;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 1.0, kEps);
  EXPECT_NEAR(r.y(), 1.0, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Center shifted by half a pixel in y.
  patch << 3.25, 2.25, 3.25, 1.25, 0.25, 1.25, 1.25, 0.25, 1.25;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.5, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Center shifted by half a pixel in x and y.
  patch << 4.5, 2.5, 2.5, 2.5, 0.5, 0.5, 2.5, 0.5, 0.5;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.5, kEps);
  EXPECT_NEAR(r.y(), 0.5, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Center outside the patch.
  patch << 18, 13, 10, 13, 8, 5, 10, 5, 2;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 2.0, kEps);
  EXPECT_NEAR(r.y(), 2.0, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);
}

TEST(EstimatorTest, Explicit_c) {
  Mat3d patch;
  Vec2d r;
  Mat2d A;
  double c;

  patch << 4.5, 2.5, 2.5, 2.5, 0.5, 0.5, 2.5, 0.5, 0.5;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.5, kEps);
  EXPECT_NEAR(r.y(), 0.5, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Function value shifted by a constant.
  patch << 14.5, 12.5, 12.5, 12.5, 10.5, 10.5, 12.5, 10.5, 10.5;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.5, kEps);
  EXPECT_NEAR(r.y(), 0.5, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 10.0, kEps);
}

TEST(EstimatorTest, Explicit_A) {
  Mat3d patch;
  Vec2d r;
  Mat2d A;
  double c;

  // Simple local minimum.
  patch << 2, 1, 2, 1, 0, 1, 2, 1, 2;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Anisotropic A.
  patch << 3, 1, 3, 2, 0, 2, 3, 1, 3;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, 2.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Anisotropic A rotated by 45 degrees CCW.
  patch << 2.0, 1.5, 4.0, 1.5, 0.0, 1.5, 4.0, 1.5, 2.0;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, 1.5, -0.5, 1.5);
  EXPECT_NEAR(c, 0.0, kEps);

  // Anisotropic A rotated by 45 degrees CW.
  patch << 4.0, 1.5, 2.0, 1.5, 0.0, 1.5, 2.0, 1.5, 4.0;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, 1.5, 0.5, 1.5);
  EXPECT_NEAR(c, 0.0, kEps);

  // Local maximum.
  patch << 0, 1, 0, 1, 2, 1, 0, 1, 0;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, -1.0, 0.0, -1.0);
  EXPECT_NEAR(c, 2.0, kEps);

  // Saddle point.
  patch << 0, 1, 0, -1, 0, -1, 0, 1, 0;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.0, kEps);
  EXPECT_NEAR(r.y(), 0.0, kEps);
  ExpectA(A, -1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);
}

TEST(EstimatorTest, Explicit_dxdy) {
  Mat3d patch;
  Vec2d r;
  Mat2d A;
  double c;

  patch << 4.5, 2.5, 2.5, 2.5, 0.5, 0.5, 2.5, 0.5, 0.5;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.5, kEps);
  EXPECT_NEAR(r.y(), 0.5, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Scaled in x.
  patch << 4.5, 2.5, 2.5, 2.5, 0.5, 0.5, 2.5, 0.5, 0.5;
  r = QuadraticFit2D(patch, 0.1, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0.05, kEps);
  EXPECT_NEAR(r.y(), 0.5, kEps);
  ExpectA(A, 100.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Sclaed in x and y.
  patch << 4.5, 2.5, 2.5, 2.5, 0.5, 0.5, 2.5, 0.5, 0.5;
  r = QuadraticFit2D(patch, 0.1, 0.1, &A, &c);
  EXPECT_NEAR(r.x(), 0.05, kEps);
  EXPECT_NEAR(r.y(), 0.05, kEps);
  ExpectA(A, 100.0, 0.0, 100.0);
  EXPECT_NEAR(c, 0.0, kEps);
}

TEST(EstimatorTest, Explicit_approximate) {
  Mat3d patch;
  Vec2d r;
  Mat2d A;
  double c;

  patch << 2, 1, 2, 1, 0, 1, 2, 1, 2;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0, kEps);
  EXPECT_NEAR(r.y(), 0, kEps);
  ExpectA(A, 1.0, 0.0, 1.0);
  EXPECT_NEAR(c, 0.0, kEps);

  // Inexact quadratic form, needs least square fit.
  patch << 5, 1, 5, 1, 0, 1, 5, 1, 5;
  r = QuadraticFit2D(patch, 1.0, 1.0, &A, &c);
  EXPECT_NEAR(r.x(), 0, kEps);
  EXPECT_NEAR(r.y(), 0, kEps);
  ExpectA(A, 3.0, 0.0, 3.0);
  EXPECT_NEAR(c, -1.3333333333333333, kEps);
}

TEST(EstimatorTest, Constructed) {
  Mat3d patch;
  Vec2d r;
  Mat2d A = Mat2d::Zero();
  const double c = 5.0;
  Vec2d r_hat;
  Mat2d A_hat;
  double c_hat;

  const double dx = 1.2;
  const double dy = 0.8;

  // Loop over the parameter grid.
  for (int xi = -5; xi <= 5; ++xi) {
    r.x() = xi * 0.3;
    for (int yi = -5; yi <= 5; ++yi) {
      r.y() = yi * 0.3;
      for (int a0i = -5; a0i <= 5; ++a0i) {
        if (a0i == 0) continue;
        A(0, 0) = a0i * 0.3;
        for (int a1i = -5; a1i <= 5; ++a1i) {
          if (a1i == 0) continue;
          A(1, 1) = a1i * 0.3;

          // Generate the patch according to the current A and r.
          for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
              const Vec2d x((col - 1) * dx, (row - 1) * dy);
              patch(row, col) = (x - r).transpose() * A * (x - r) + c;
            }
          }

          // Estimate and verify A and r.
          r_hat = QuadraticFit2D(patch, dx, dy, &A_hat, &c_hat);
          EXPECT_NEAR(r_hat.x(), r.x(), kEps);
          EXPECT_NEAR(r_hat.y(), r.y(), kEps);
          ExpectA(A_hat, A(0, 0), A(0, 1), A(1, 1));
          EXPECT_NEAR(c_hat, c, kEps);
        }
      }
    }
  }
}

}  // namespace
}  // namespace qcraft
