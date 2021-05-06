#include "onboard/math/line_fitter.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kEps = 1e-14;

TEST(LineFitterTest, SingleEvaluationTest) {
  std::vector<Vec2d> data;
  std::vector<double> weights;

  data = {Vec2d(0, 0), Vec2d(1, 1), Vec2d(2, 2), Vec2d(3, 3)};
  weights = {1.0, 1.0, 1.0, 1.0};
  LineFitter fitter(data, weights, true);
  fitter.FitData(FITTER::DEMING, true, true);
  EXPECT_NEAR(fitter.mse(), 0.0, kEps);
  EXPECT_NEAR(fitter.FitPointError(Vec2d(-1.0, -1.0)), 0.0, kEps);

  data = {Vec2d(0, 0), Vec2d(0, 1), Vec2d(0, 2)};
  weights = {1.0, 1.0, 1.0};
  fitter.LoadData(data, weights);
  fitter.FitData();
  fitter.FitData(FITTER::DEMING, true, true);
  EXPECT_NEAR(abs(fitter.tangent()[1]), 1.0, kEps);

  data = {Vec2d(0, 0), Vec2d(1, 0), Vec2d(2, 0)};
  weights = {1.0, 1.0, 1.0};
  fitter.LoadData(data, weights);
  fitter.FitData();
  fitter.FitData(FITTER::DEMING, true, true);
  EXPECT_NEAR(abs(fitter.tangent()[0]), 1.0, kEps);

  data = {Vec2d(0, 0), Vec2d(2, 1), Vec2d(1, 2), Vec2d(3, 3)};
  weights = {1.0, 1.0, 1.0, 1.0};
  fitter.LoadData(data, weights);
  fitter.FitData(FITTER::DEMING, true, true);
  EXPECT_NEAR(fitter.tangent()[0] - fitter.tangent()[1], 0.0, kEps);
}

}  // namespace
}  // namespace qcraft
