#include "onboard/planner/test_util/perception_object_builder.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PerceptionObjectBuilder, Build) {
  const auto obj = PerceptionObjectBuilder().Build();
  EXPECT_THAT(obj.accel(), ProtoEqText("x: 1.0, y: 0.0"));
  EXPECT_EQ(obj.id(), "object");
  EXPECT_EQ(obj.bounding_box().length(), 4.0);
  EXPECT_EQ(obj.bounding_box().width(), 2.0);
  EXPECT_EQ(obj.contour_size(), 4);
}

TEST(PerceptionObjectBuilder, CustomizedBuild) {
  const auto obj = PerceptionObjectBuilder()
                       .set_id("o1")
                       .set_timestamp(10.0)
                       .set_accel(2.0)
                       .set_velocity(2.0)
                       .Build();
  EXPECT_NEAR(obj.vel().x(), 2.0, 1e-6);
  EXPECT_NEAR(obj.vel().y(), 0.0, 1e-6);
  EXPECT_EQ(obj.timestamp(), 10.0);
  EXPECT_NEAR(obj.accel().x(), 2.0, 1e-6);
  EXPECT_NEAR(obj.accel().y(), 0.0, 1e-6);
  EXPECT_EQ(obj.id(), "o1");
  EXPECT_EQ(obj.bounding_box().length(), 4.0);
  EXPECT_EQ(obj.bounding_box().width(), 2.0);
  EXPECT_EQ(obj.contour_size(), 4);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft
