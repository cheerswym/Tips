#include "onboard/math/polynomial_fitter.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kEps = 1e-14;

TEST(PolynomialFitterTest, SingleEvaluationTest) {
  std::vector<Vec2d> data;
  std::vector<double> weights;

  // y = x
  data = {Vec2d(0, 0), Vec2d(1, 1), Vec2d(2, 2), Vec2d(3, 3)};
  weights = {1.0, 1.0, 1.0, 1.0};
  PolynomialFitter fitter(2, data, weights, true);
  fitter.FitData(LS_SOLVER::QR, true);
  EXPECT_NEAR(fitter.mse(), 0.0, kEps);
  EXPECT_NEAR(fitter.Evaluate(-1), -1, kEps);

  // y = x^2;
  data = {Vec2d(0, 0), Vec2d(1, 1), Vec2d(2, 4)};
  weights = {1.0, 1.0, 1.0};
  fitter.LoadData(data, weights);
  fitter.FitData(LS_SOLVER::QR, true);
  EXPECT_NEAR(fitter.mse(), 0.0, kEps);
  EXPECT_NEAR(fitter.Evaluate(-1), 1, kEps);

  // y = x^3;
  data = {Vec2d(0, 0), Vec2d(1, 1), Vec2d(2, 8), Vec2d(-2, -8)};
  weights = {1.0, 1.0, 1.0, 1.0};
  fitter.SetDegree(3);
  fitter.LoadData(data, weights);
  fitter.FitData(LS_SOLVER::QR, true);
  EXPECT_NEAR(fitter.mse(), 0.0, kEps);
  EXPECT_NEAR(fitter.Evaluate(-1), -1, kEps);

  // y = x^2;
  data = {Vec2d(0, 0), Vec2d(1, 1), Vec2d(-2, 4), Vec2d(-1, 1), Vec2d(10, 100)};
  weights = {0.1, 1.0, 1.9, 0.3, 100};
  fitter.SetDegree(4);
  fitter.LoadData(data, weights);
  fitter.FitData(LS_SOLVER::QR, true);
  EXPECT_NEAR(fitter.mse(), 0.0, kEps);
  EXPECT_NEAR(fitter.Evaluate(-1), 1, kEps);
}

}  // namespace
}  // namespace qcraft
