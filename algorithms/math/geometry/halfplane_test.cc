#include "onboard/math/geometry/halfplane.h"

#include "gtest/gtest.h"
#include "onboard/math/test_util.h"

namespace qcraft {

namespace {

TEST(HalfPlaneTest, Horizontal) {
  HalfPlane hp({1.0, 0.0}, {2.0, 0.0});
  EXPECT_THAT(hp.start(), Vec2dEqXY(1.0, 0.0));
  EXPECT_THAT(hp.end(), Vec2dEqXY(2.0, 0.0));
  EXPECT_EQ(hp.lon_proj({3.0, 0.0}), 2.0);
  EXPECT_EQ(hp.lon_proj({0.0, 1.0}), -1.0);
  EXPECT_EQ(hp.lat_proj({3.0, 0.0}), 0.0);
  EXPECT_EQ(hp.lat_proj({0.0, 1.0}), 1.0);
  EXPECT_THAT(hp.tangent(), Vec2dNearXY(1.0, 0.0, 1e-8));
  EXPECT_THAT(hp.Transform(Vec2d(5.0, 6.0)), Vec2dNearXY(4.0, 6.0, 1e-8));
  EXPECT_THAT(hp.InvTransform(Vec2d(5.0, 6.0)), Vec2dNearXY(6.0, 6.0, 1e-8));
}

TEST(HalfPlaneTest, Vertical) {
  HalfPlane hp({1.0, 0.0}, {1.0, 1.0});
  EXPECT_THAT(hp.start(), Vec2dEqXY(1.0, 0.0));
  EXPECT_THAT(hp.end(), Vec2dEqXY(1.0, 1.0));
  EXPECT_EQ(hp.lon_proj({3.0, 0.0}), 0.0);
  EXPECT_EQ(hp.lon_proj({0.0, 1.0}), 1.0);
  EXPECT_EQ(hp.lat_proj({3.0, 0.0}), -2.0);
  EXPECT_EQ(hp.lat_proj({0.0, 1.0}), 1.0);
  EXPECT_THAT(hp.tangent(), Vec2dNearXY(0.0, 1.0, 1e-8));
  EXPECT_THAT(hp.Transform(Vec2d(5.0, 6.0)), Vec2dNearXY(6.0, -4.0, 1e-8));
  EXPECT_THAT(hp.InvTransform(Vec2d(5.0, 6.0)), Vec2dNearXY(-5.0, 5.0, 1e-8));
}
}  // namespace
}  // namespace qcraft
