#include "onboard/math/cubic_spline.h"

#include <cmath>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace {

TEST(CubicSplineTest, BasicTest) {
  constexpr double kEps = 1e-3;
  std::vector<double> x = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> y = {0.0, 2.0, 4.0, 2.0, 0.0};
  CubicSpline spline(std::move(x), std::move(y),
                     {.type = CubicSpline::BoundaryType::FOD, .value = 3.0},
                     {.type = CubicSpline::BoundaryType::SOD, .value = 0.0});

  EXPECT_NEAR(spline.Evaluate(0.0), 0.0, kEps);
  EXPECT_NEAR(spline.Evaluate(1.0), 2.0, kEps);
  EXPECT_NEAR(spline.Evaluate(2.0), 4.0, kEps);
  EXPECT_NEAR(spline.Evaluate(3.0), 2.0, kEps);
  EXPECT_NEAR(spline.Evaluate(4.0), 0.0, kEps);

  EXPECT_NEAR(spline.EvaluateDerivative(1, 0.0), 3.0, kEps);
  EXPECT_NEAR(spline.EvaluateDerivative(2, 4.0), 0.0, kEps);
}

TEST(CubicSplineTest, TestTwoPoints1) {
  constexpr double kEps = 1e-3;
  std::vector<double> x = {0.0, 2.0};
  std::vector<double> y = {0.0, 4.0};
  CubicSpline spline(std::move(x), std::move(y),
                     {.type = CubicSpline::BoundaryType::FOD, .value = 3.0},
                     {.type = CubicSpline::BoundaryType::SOD, .value = 0.0});

  EXPECT_NEAR(spline.Evaluate(0.0), 0.0, kEps);
  EXPECT_NEAR(spline.Evaluate(2.0), 4.0, kEps);

  EXPECT_NEAR(spline.EvaluateDerivative(1, 0.0), 3.0, kEps);
  EXPECT_NEAR(spline.EvaluateDerivative(2, 2.0), 0.0, kEps);
}

TEST(CubicSplineTest, TestTwoPoints2) {
  constexpr double kEps = 1e-3;
  std::vector<double> x = {0.0, 2.0};
  std::vector<double> y = {0.0, 4.0};
  CubicSpline spline(std::move(x), std::move(y),
                     {.type = CubicSpline::BoundaryType::SOD, .value = 3.0},
                     {.type = CubicSpline::BoundaryType::FOD, .value = 1.0});

  EXPECT_NEAR(spline.Evaluate(0.0), 0.0, kEps);
  EXPECT_NEAR(spline.Evaluate(2.0), 4.0, kEps);

  EXPECT_NEAR(spline.EvaluateDerivative(2, 0.0), 3.0, kEps);
  EXPECT_NEAR(spline.EvaluateDerivative(1, 2.0), 1.0, kEps);
}

}  // namespace
}  // namespace qcraft
