#include "onboard/math/geometry/util.h"

#include <limits>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {
namespace {

using testing::FloatNear;
using testing::Pair;

TEST(BoxProjection, Works) {
  const Box2d box1(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/4.0,
                   /*width=*/2.0);
  EXPECT_THAT(
      ProjectBoxToRay(/*pos=*/Vec2d(0.0, 0.0), /*dir=*/Vec2d(1.0, 0.0), box1),
      Pair(-2.0, 2.0));
  EXPECT_THAT(
      ProjectBoxToRay(/*pos=*/Vec2d(0.0, 0.0), /*dir=*/Vec2d(0.0, 1.0), box1),
      Pair(-1.0, 1.0));

  const Box2d box2(Vec2d(1.0, 2.0), /*heading=*/0.0, /*length=*/4.0,
                   /*width=*/2.0);
  EXPECT_THAT(
      ProjectBoxToRay(/*pos=*/Vec2d(0.0, 0.0), /*dir=*/Vec2d(1.0, 0.0), box2),
      Pair(-1.0, 3.0));
  EXPECT_THAT(ProjectBoxToRay(/*pos=*/Vec2d(0.0, 0.0),
                              /*dir=*/Vec2d(0.0, 1.0), box2),
              Pair(1.0, 3.0));

  EXPECT_THAT(ProjectBoxToRay(/*pos=*/Vec2d(0.0, 1.0),
                              /*dir=*/Vec2d(0.0, 1.0), box2),
              Pair(0.0, 2.0));

  // Check with each corner point's projection of an arbitrary box.
  const Box2d box3(Vec2d(-1.0, 3.4), /*heading=*/M_PI * 0.25, /*length=*/10.0,
                   /*width=*/3.0);
  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();
  const Vec2d ray_center = Vec2d(-5.0, -4.0);
  const Vec2d ray_dir = Vec2d::UnitFromAngle(10.0);
  for (const auto& pt : box3.GetCornersCounterClockwise()) {
    const double proj = ray_dir.Dot(pt - ray_center);
    UpdateMax(proj, &max_proj);
    UpdateMin(proj, &min_proj);
  }

  const auto result = ProjectBoxToRay(ray_center, ray_dir, box3);
  EXPECT_NEAR(result.first, min_proj, 1e-8);
  EXPECT_NEAR(result.second, max_proj, 1e-8);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft
