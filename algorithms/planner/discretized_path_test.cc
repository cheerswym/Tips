#include "onboard/planner/discretized_path.h"

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/math/frenet_frame.h"

namespace qcraft {
namespace planner {

TEST(DiscretizedPathTest, TestStraightPath) {
  constexpr double kEps = 1.0e-6;

  PathPoint p0;
  p0.set_x(0.0);
  p0.set_y(0.0);
  p0.set_z(0.0);
  p0.set_theta(0.0);
  p0.set_kappa(0.0);
  p0.set_lambda(0.0);
  p0.set_s(0.0);

  PathPoint p1;
  p1.set_x(1.0);
  p1.set_y(0.0);
  p1.set_z(0.0);
  p1.set_theta(0.0);
  p1.set_kappa(0.0);
  p0.set_lambda(0.0);
  p1.set_s(1.0);

  PathPoint p2;
  p2.set_x(2.0);
  p2.set_y(0.0);
  p2.set_z(0.0);
  p2.set_theta(0.0);
  p2.set_kappa(0.0);
  p0.set_lambda(0.0);
  p2.set_s(2.0);

  std::vector<PathPoint> path_points = {p0, p1, p2};

  DiscretizedPath path(path_points);

  auto frenet_path =
      BuildBruteForceFrenetFrame(
          {Vec2d(p0.x(), p0.y()), Vec2d(p1.x(), p1.y()), Vec2d(p2.x(), p2.y())})
          .value();

  PathPoint p;

  p = path.Evaluate(0.0);
  EXPECT_EQ(p.x(), 0.0);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 0.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(0.6);
  EXPECT_EQ(p.x(), 0.6);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 0.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(1.0);
  EXPECT_EQ(p.x(), 1.0);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 0.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(1.6);
  EXPECT_EQ(p.x(), 1.6);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 0.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(2.0);
  EXPECT_EQ(p.x(), 2.0);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 0.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(2.5);
  EXPECT_EQ(p.x(), 2.0);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 0.0);
  EXPECT_EQ(p.lambda(), 0.0);

  Vec2d pt;

  pt = Vec2d(-0.5, 0.1);
  EXPECT_NEAR(frenet_path.XYToSL(pt).s, path.XYToSL(pt).s, kEps);
  EXPECT_NEAR(frenet_path.XYToSL(pt).l, path.XYToSL(pt).l, kEps);

  pt = Vec2d(-0.5, -0.1);
  EXPECT_NEAR(frenet_path.XYToSL(pt).s, path.XYToSL(pt).s, kEps);
  EXPECT_NEAR(frenet_path.XYToSL(pt).l, path.XYToSL(pt).l, kEps);

  pt = Vec2d(1.5, 0.1);
  EXPECT_NEAR(frenet_path.XYToSL(pt).s, path.XYToSL(pt).s, kEps);
  EXPECT_NEAR(frenet_path.XYToSL(pt).l, path.XYToSL(pt).l, kEps);

  pt = Vec2d(1.5, -0.1);
  EXPECT_NEAR(frenet_path.XYToSL(pt).s, path.XYToSL(pt).s, kEps);
  EXPECT_NEAR(frenet_path.XYToSL(pt).l, path.XYToSL(pt).l, kEps);
}

TEST(DiscretizedPathTest, TestCircularPath) {
  constexpr double kEps = 1.0e-6;

  PathPoint p0;
  p0.set_x(0.0);
  p0.set_y(0.0);
  p0.set_z(0.0);
  p0.set_theta(0.0);
  p0.set_kappa(1.0);
  p0.set_lambda(0.0);
  p0.set_s(0.0);

  PathPoint p1;
  p1.set_x(1.0);
  p1.set_y(1.0);
  p1.set_z(0.0);
  p1.set_theta(M_PI_2);
  p1.set_kappa(1.0);
  p0.set_lambda(0.0);
  p1.set_s(M_PI_2);

  PathPoint p2;
  p2.set_x(0.0);
  p2.set_y(2.0);
  p2.set_z(0.0);
  p2.set_theta(M_PI);
  p2.set_kappa(1.0);
  p0.set_lambda(0.0);
  p2.set_s(M_PI);

  std::vector<PathPoint> path_points = {p0, p1, p2};

  DiscretizedPath path(path_points);

  PathPoint p;

  p = path.Evaluate(0.0);
  EXPECT_EQ(p.x(), 0.0);
  EXPECT_EQ(p.y(), 0.0);
  EXPECT_EQ(p.theta(), 0.0);
  EXPECT_EQ(p.kappa(), 1.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(M_PI_4);
  EXPECT_NEAR(p.x(), 0.5, kEps);
  EXPECT_NEAR(p.theta(), M_PI_4, kEps);
  EXPECT_NEAR(p.theta(), M_PI_4, kEps);
  EXPECT_EQ(p.kappa(), 1.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(M_PI_2);
  EXPECT_EQ(p.x(), 1.0);
  EXPECT_EQ(p.y(), 1.0);
  EXPECT_EQ(p.theta(), M_PI_2);
  EXPECT_EQ(p.kappa(), 1.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(M_PI_4 * 3.0);
  EXPECT_NEAR(p.x(), 0.5, kEps);
  EXPECT_NEAR(p.y(), 1.5, kEps);
  EXPECT_NEAR(p.theta(), M_PI_4 * 3.0, kEps);
  EXPECT_EQ(p.kappa(), 1.0);
  EXPECT_EQ(p.lambda(), 0.0);

  p = path.Evaluate(M_PI);
  EXPECT_EQ(p.x(), 0.0);
  EXPECT_EQ(p.y(), 2.0);
  EXPECT_EQ(p.theta(), M_PI);
  EXPECT_EQ(p.kappa(), 1.0);
  EXPECT_EQ(p.lambda(), 0.0);
}

}  // namespace planner
}  // namespace qcraft
