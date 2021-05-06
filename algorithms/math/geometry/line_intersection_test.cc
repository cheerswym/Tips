#include "onboard/math/geometry/line_intersection.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

constexpr double kEps = 1e-14;

// Computes the intersection between two lines, each specified by two points.
// Returns the intersection point x.
// Returns the interpolation factor of the intersection point x along line0 as
// s0, i.e. x = Lerp(line0_p0, line0_p1, s0).
// Returns the interpolation factor of x along line1 as s1.
Vec2d FindIntersectionBetweenLinesWithPoints(Vec2d line0_p0, Vec2d line0_p1,
                                             Vec2d line1_p0, Vec2d line1_p1,
                                             double *s0 = nullptr,
                                             double *s1 = nullptr) {
  DCHECK(s0 != nullptr);
  DCHECK(s1 != nullptr);
  return FindIntersectionBetweenLinesWithTangents(
      line0_p0, line0_p1 - line0_p0, line1_p0, line1_p1 - line1_p0, s0, s1);
}

TEST(LineIntersectionTest, PointsTest) {
  double s0, s1;
  Vec2d x = FindIntersectionBetweenLinesWithPoints(
      {0.0, 1.0}, {2.0, 1.0}, {1.0, 0.0}, {1.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.5, kEps);
  EXPECT_NEAR(s1, 0.5, kEps);

  x = FindIntersectionBetweenLinesWithPoints({1.0, 1.0}, {2.0, 1.0}, {1.0, 0.0},
                                             {1.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.0, kEps);
  EXPECT_NEAR(s1, 0.5, kEps);

  x = FindIntersectionBetweenLinesWithPoints({0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0},
                                             {1.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 1.0, kEps);
  EXPECT_NEAR(s1, 0.5, kEps);

  x = FindIntersectionBetweenLinesWithPoints({0.0, 1.0}, {2.0, 1.0}, {1.0, 1.0},
                                             {1.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.5, kEps);
  EXPECT_NEAR(s1, 0.0, kEps);

  x = FindIntersectionBetweenLinesWithPoints({1.0, 1.0}, {2.0, 1.0}, {1.0, 1.0},
                                             {1.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.0, kEps);
  EXPECT_NEAR(s1, 0.0, kEps);

  x = FindIntersectionBetweenLinesWithPoints({2.0, 1.0}, {3.0, 1.0}, {1.0, 2.0},
                                             {1.0, 3.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, -1.0, kEps);
  EXPECT_NEAR(s1, -1.0, kEps);
}

TEST(LineIntersectionTest, TangentsTest) {
  double s0, s1;
  Vec2d x = FindIntersectionBetweenLinesWithTangents(
      {0.0, 1.0}, {2.0, 0.0}, {1.0, 0.0}, {0.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.5, kEps);
  EXPECT_NEAR(s1, 0.5, kEps);

  x = FindIntersectionBetweenLinesWithTangents(
      {1.0, 1.0}, {1.0, 0.0}, {1.0, 0.0}, {0.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.0, kEps);
  EXPECT_NEAR(s1, 0.5, kEps);

  x = FindIntersectionBetweenLinesWithTangents(
      {0.0, 1.0}, {1.0, 0.0}, {1.0, 0.0}, {0.0, 2.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 1.0, kEps);
  EXPECT_NEAR(s1, 0.5, kEps);

  x = FindIntersectionBetweenLinesWithTangents(
      {0.0, 1.0}, {2.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.5, kEps);
  EXPECT_NEAR(s1, 0.0, kEps);

  x = FindIntersectionBetweenLinesWithTangents(
      {1.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, 0.0, kEps);
  EXPECT_NEAR(s1, 0.0, kEps);

  x = FindIntersectionBetweenLinesWithTangents(
      {2.0, 1.0}, {1.0, 0.0}, {1.0, 2.0}, {0.0, 1.0}, &s0, &s1);
  EXPECT_NEAR(x.x(), 1.0, kEps);
  EXPECT_NEAR(x.y(), 1.0, kEps);
  EXPECT_NEAR(s0, -1.0, kEps);
  EXPECT_NEAR(s1, -1.0, kEps);
}

TEST(LineIntersectionTest, DegeneracyTest) {
  double s0, s1;
  Vec2d x = FindIntersectionBetweenLinesWithPoints(
      {0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, &s0, &s1);

  x = FindIntersectionBetweenLinesWithPoints({0.0, 0.0}, {2.0, 0.0}, {1.0, 0.0},
                                             {3.0, 0.0}, &s0, &s1);
}

}  // namespace
}  // namespace qcraft
