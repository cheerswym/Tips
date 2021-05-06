#include "onboard/math/geometry/bezier_curve.h"

#include <array>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/math/test_util.h"

namespace qcraft::planner {
namespace {

TEST(BezierCurve, TestFifthOrderBezier) {
  const std::array<Vec2d, 6> control_points = {
      Vec2d(0.0, 0.0), Vec2d(0.0, 1.0), Vec2d(1.0, 2.0),
      Vec2d(2.0, 2.0), Vec2d(3.0, 1.0), Vec2d(3.0, 0.0)};

  EXPECT_THAT(Bezier::SampleBezier(control_points, 0.0), Vec2dEqXY(0.0, 0.0));
  EXPECT_THAT(Bezier::SampleBezier(control_points, 1.0), Vec2dEqXY(3.0, 0.0));
  EXPECT_THAT(Bezier::SampleBezierTangent(control_points, 0.0),
              Vec2dEqXY(0.0, 1.0));
  EXPECT_THAT(Bezier::SampleBezierTangent(control_points, 1.0),
              Vec2dEqXY(0.0, -1.0));
}

TEST(BezierCurve, TestFifthOrderBezierWithPrecomputedCoeffs) {
  const std::array<Vec2d, 6> control_points = {
      Vec2d(0.0, 0.0), Vec2d(0.0, 1.0), Vec2d(1.0, 2.0),
      Vec2d(2.0, 2.0), Vec2d(3.0, 1.0), Vec2d(3.0, 0.0)};
  auto binomial_coeffs = Bezier::GetBinomialCoeff(5);
  EXPECT_THAT(Bezier::SampleBezierWithBinomialCoeffs(control_points,
                                                     binomial_coeffs, 0.0),
              Vec2dEqXY(0.0, 0.0));
  EXPECT_THAT(Bezier::SampleBezierWithBinomialCoeffs(control_points,
                                                     binomial_coeffs, 1.0),
              Vec2dEqXY(3.0, 0.0));

  binomial_coeffs = Bezier::GetBinomialCoeff(4);
  EXPECT_THAT(Bezier::SampleBezierTangentWithBinomialCoeffs(
                  control_points, binomial_coeffs, 0.0),
              Vec2dEqXY(0.0, 1.0));
  EXPECT_THAT(Bezier::SampleBezierTangentWithBinomialCoeffs(
                  control_points, binomial_coeffs, 1.0),
              Vec2dEqXY(0.0, -1.0));
}

TEST(BezierCurve, TestThirdOrderBezier) {
  std::array<Vec2d, 4> control_points = {Vec2d(0.0, 0.0), Vec2d(0.0, 1.0),
                                         Vec2d(1.0, 2.0), Vec2d(2.0, 2.0)};

  EXPECT_THAT(Bezier::SampleThirdOrderBezier(control_points, 0.0),
              Vec2dEqXY(0.0, 0.0));
  EXPECT_THAT(Bezier::SampleThirdOrderBezier(control_points, 1.0),
              Vec2dEqXY(2.0, 2.0));
  EXPECT_THAT(Bezier::SampleThirdOrderBezierTangent(control_points, 0.0),
              Vec2dEqXY(0.0, 1.0));
  EXPECT_THAT(Bezier::SampleThirdOrderBezierTangent(control_points, 1.0),
              Vec2dEqXY(1.0, 0.0));
}

TEST(BezierCurve, SplitBezier) {
  constexpr double kEpsilon = 1e-4;
  std::vector<Vec2d> control_points(
      {{120, 160}, {35, 200}, {220, 260}, {220, 40}});

  const auto [split_05_left, split_05_right] =
      Bezier::SplitBezier(control_points, 0.5);
  EXPECT_NEAR(split_05_left[0].x(), 120.0, kEpsilon);
  EXPECT_NEAR(split_05_left[0].y(), 160.0, kEpsilon);
  EXPECT_NEAR(split_05_left[1].x(), 77.5, kEpsilon);
  EXPECT_NEAR(split_05_left[1].y(), 180.0, kEpsilon);
  EXPECT_NEAR(split_05_left[2].x(), 102.5, kEpsilon);
  EXPECT_NEAR(split_05_left[2].y(), 205.0, kEpsilon);
  EXPECT_NEAR(split_05_left[3].x(), 138.125, kEpsilon);
  EXPECT_NEAR(split_05_left[3].y(), 197.5, kEpsilon);
  EXPECT_NEAR(split_05_right[0].x(), 220.0, kEpsilon);
  EXPECT_NEAR(split_05_right[0].y(), 40.0, kEpsilon);
  EXPECT_NEAR(split_05_right[1].x(), 220.0, kEpsilon);
  EXPECT_NEAR(split_05_right[1].y(), 150.0, kEpsilon);
  EXPECT_NEAR(split_05_right[2].x(), 173.75, kEpsilon);
  EXPECT_NEAR(split_05_right[2].y(), 190.0, kEpsilon);
  EXPECT_NEAR(split_05_right[3].x(), 138.125, kEpsilon);
  EXPECT_NEAR(split_05_right[3].y(), 197.5, kEpsilon);
  EXPECT_EQ(split_05_left.size(), control_points.size());
  EXPECT_EQ(split_05_right.size(), control_points.size());

  const auto [split_08_left, split_08_right] =
      Bezier::SplitBezier(control_points, 0.8f);
  EXPECT_NEAR(split_08_left[0].x(), 120.0, kEpsilon);
  EXPECT_NEAR(split_08_left[0].y(), 160.0, kEpsilon);
  EXPECT_NEAR(split_08_left[1].x(), 52.0, kEpsilon);
  EXPECT_NEAR(split_08_left[1].y(), 192.0, kEpsilon);
  EXPECT_NEAR(split_08_left[2].x(), 156.8, kEpsilon);
  EXPECT_NEAR(split_08_left[2].y(), 236.8, kEpsilon);
  EXPECT_NEAR(split_08_left[3].x(), 201.44, kEpsilon);
  EXPECT_NEAR(split_08_left[3].y(), 140.8, kEpsilon);
  EXPECT_NEAR(split_08_right[0].x(), 220.0, kEpsilon);
  EXPECT_NEAR(split_08_right[0].y(), 40.0, kEpsilon);
  EXPECT_NEAR(split_08_right[1].x(), 220.0, kEpsilon);
  EXPECT_NEAR(split_08_right[1].y(), 84.0, kEpsilon);
  EXPECT_NEAR(split_08_right[2].x(), 212.6, kEpsilon);
  EXPECT_NEAR(split_08_right[2].y(), 116.8, kEpsilon);
  EXPECT_NEAR(split_08_right[3].x(), 201.44, kEpsilon);
  EXPECT_NEAR(split_08_right[3].y(), 140.8, kEpsilon);
  EXPECT_EQ(split_08_left.size(), control_points.size());
  EXPECT_EQ(split_08_right.size(), control_points.size());

  const auto [split_033_left, split_033_right] =
      Bezier::SplitBezier(control_points, 0.33f);
  EXPECT_NEAR(split_033_left[0].x(), 120.0, kEpsilon);
  EXPECT_NEAR(split_033_left[0].y(), 160.0, kEpsilon);
  EXPECT_NEAR(split_033_left[1].x(), 91.950, kEpsilon);
  EXPECT_NEAR(split_033_left[1].y(), 173.2, kEpsilon);
  EXPECT_NEAR(split_033_left[2].x(), 93.303, kEpsilon);
  EXPECT_NEAR(split_033_left[2].y(), 188.578, kEpsilon);
  EXPECT_NEAR(split_033_left[3].x(), 107.7077, kEpsilon);
  EXPECT_NEAR(split_033_left[3].y(), 195.3529, kEpsilon);
  EXPECT_NEAR(split_033_right[0].x(), 220.0, kEpsilon);
  EXPECT_NEAR(split_033_right[0].y(), 40.0, kEpsilon);
  EXPECT_NEAR(split_033_right[1].x(), 220.0, kEpsilon);
  EXPECT_NEAR(split_033_right[1].y(), 187.4, kEpsilon);
  EXPECT_NEAR(split_033_right[2].x(), 136.9535, kEpsilon);
  EXPECT_NEAR(split_033_right[2].y(), 209.108, kEpsilon);
  EXPECT_NEAR(split_033_right[3].x(), 107.7077, kEpsilon);
  EXPECT_NEAR(split_033_right[3].y(), 195.3529, kEpsilon);
  EXPECT_EQ(split_033_left.size(), control_points.size());
  EXPECT_EQ(split_033_right.size(), control_points.size());
}
}  // namespace
}  // namespace qcraft::planner
