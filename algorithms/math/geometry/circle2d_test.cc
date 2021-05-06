#include "onboard/math/geometry/circle2d.h"

#include "onboard/math/test_util.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(Circle2d, Properties) {
  Circle2d c(Vec2d(1.0, 2.0), 3.0);
  EXPECT_THAT(c.center(), Vec2dEqXY(1.0, 2.0));
  EXPECT_EQ(c.radius(), 3.0);
}

TEST(Circle2d, Inside) {
  Circle2d c(Vec2d(1.0, 2.0), 3.0);
  EXPECT_FALSE(c.inside(Vec2d(0.0, 10.0)));
  EXPECT_TRUE(c.inside(Vec2d(1.0, 2.0)));
  EXPECT_TRUE(c.inside(Vec2d(2.0, 3.0)));
}
}  // namespace
}  // namespace qcraft::planner
