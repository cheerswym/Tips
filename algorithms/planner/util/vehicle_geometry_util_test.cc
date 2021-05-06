#include "onboard/planner/util/vehicle_geometry_util.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/test_util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {
TEST(GetAVBox, WithoutBuffer) {
  const auto vehicle_geom = DefaultVehicleGeometry();
  {
    const auto box = GetAvBox(Vec2d(0.0, 0.0), /*av_theta=*/0.0, vehicle_geom);
    EXPECT_THAT(box.center(), Vec2dNearXY(1.5, 0.0, 1e-6));
    EXPECT_EQ(box.heading(), 0.0);
  }
  {
    const auto box =
        GetAvBox(Vec2d(0.0, 1.0), /*av_theta=*/M_PI_2, vehicle_geom);
    EXPECT_THAT(box.center(), Vec2dNearXY(0.0, 2.5, 1e-6));
    EXPECT_EQ(box.heading(), M_PI_2);
  }
}

TEST(GetAVBox, WithBuffer) {
  const auto vehicle_geom = DefaultVehicleGeometry();
  {
    const auto box =
        GetAvBoxWithBuffer(Vec2d(0.0, 0.0), /*av_theta=*/0.0, vehicle_geom,
                           /*length_buffer=*/1.0, /*width_buffer=*/2.0);
    EXPECT_THAT(box.center(), Vec2dNearXY(1.5, 0.0, 1e-6));
    EXPECT_EQ(box.heading(), 0.0);
    EXPECT_EQ(box.length(), 7.0);
    EXPECT_EQ(box.width(), 6.0);
  }
  {
    const auto box =
        GetAvBoxWithBuffer(Vec2d(0.0, 1.0), /*av_theta=*/M_PI_2, vehicle_geom,
                           /*length_buffer=*/1.0, /*width_buffer=*/2.0);
    EXPECT_THAT(box.center(), Vec2dNearXY(0.0, 2.5, 1e-6));
    EXPECT_EQ(box.heading(), M_PI_2);
    EXPECT_EQ(box.length(), 7.0);
    EXPECT_EQ(box.width(), 6.0);
  }
}

TEST(CreateOffsetRect, DefaultVehicleGeometry) {
  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto rect = CreateOffsetRectFromVehicleGeometry(vehicle_geom);
  EXPECT_NEAR(rect.half_length(), vehicle_geom.length() * 0.5, 1e-6);
  EXPECT_NEAR(rect.half_width(), vehicle_geom.width() * 0.5, 1e-6);
  EXPECT_NEAR(rect.offset(),
              -0.5 * (vehicle_geom.front_edge_to_center() -
                      vehicle_geom.back_edge_to_center()),
              1e-6);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
