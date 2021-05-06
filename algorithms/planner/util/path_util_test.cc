#include "onboard/planner/util/path_util.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(PlannerPathUtilTest, PathPointUtilTest) {
  qcraft::PathPoint path_point;
  path_point.set_x(0.0);
  path_point.set_y(0.0);
  path_point.set_z(1.0);
  path_point.set_theta(0.0);
  path_point.set_kappa(0.0);
  path_point.set_lambda(0.0);
  path_point.set_s(0.0);
  EXPECT_EQ(ToVec2d(path_point), qcraft::Vec2d(0.0, 0.0));

  qcraft::PathPoint from;
  from.set_x(0.0);
  from.set_y(0.0);
  from.set_z(1.0);
  from.set_theta(0.0);
  from.set_kappa(0.0);
  from.set_lambda(0.0);
  from.set_s(0.0);

  qcraft::PathPoint to;
  to.set_x(0.0);
  to.set_y(0.0);
  to.set_z(1.0);
  to.set_theta(0.0);
  to.set_kappa(0.0);
  to.set_lambda(0.0);
  to.set_s(0.0);
  EXPECT_EQ(DistanceTo(from, to), 0.0);
  EXPECT_EQ(Heading(from, to), qcraft::Vec2d(0.0, 0.0));
  EXPECT_NE(Heading(from, to), qcraft::Vec2d(0.0, 5e-5));

  to.set_y(5e-5);
  EXPECT_NE(Heading(from, to), qcraft::Vec2d(0.0, 0.0));
  EXPECT_EQ(Heading(from, to), qcraft::Vec2d(0.0, 1.0));

  to.set_y(-1.0);
  EXPECT_EQ(DistanceTo(from, to), 1.0);
  EXPECT_EQ(Heading(from, to), qcraft::Vec2d(0.0, -1.0));
}

}  // namespace
}  // namespace qcraft::planner
