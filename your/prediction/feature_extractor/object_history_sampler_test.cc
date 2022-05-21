#include "onboard/prediction/feature_extractor/object_history_sampler.h"

#include "gtest/gtest.h"
#include "onboard/planner/test_util/perception_object_builder.h"

namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
TEST(ProphnetFeatureExtractorTest, LerpObjectProtoTest) {
  const Vec2d pos(60.0, 80.0);
  const double vel = 10.0;
  std::string id("1");
  const auto obj_a = planner::PerceptionObjectBuilder()
                         .set_id(id)
                         .set_type(OT_VEHICLE)
                         .set_pos(pos)
                         .set_yaw(M_PI)
                         .set_velocity(vel)
                         .set_timestamp(0.0)
                         .set_box_center(pos)
                         .set_length_width(/*length=*/2.0, /*width=*/1.0)
                         .Build();
  const auto obj_b = planner::PerceptionObjectBuilder()
                         .set_id(id)
                         .set_type(OT_VEHICLE)
                         .set_pos(pos + vel)
                         .set_yaw(-M_PI_2)
                         .set_velocity(vel)
                         .set_timestamp(1.0)
                         .set_box_center(pos)
                         .set_length_width(/*length=*/2.0, /*width=*/1.0)
                         .Build();
  const auto obj_m = LerpObjectProto(obj_a, obj_b, 0.5);
  EXPECT_EQ(obj_m.id(), obj_b.id());
  EXPECT_EQ(obj_m.type(), obj_b.type());
  EXPECT_NEAR(obj_m.timestamp(), 0.5, kEps);
  EXPECT_NEAR(obj_m.pos().x(), 65.0, kEps);
  EXPECT_NEAR(obj_m.pos().y(), 85.0, kEps);
  EXPECT_NEAR(obj_m.vel().x(), -vel * 0.5, kEps);
  EXPECT_NEAR(obj_m.vel().y(), -vel * 0.5, kEps);
  EXPECT_NEAR(obj_m.yaw(), -0.75 * M_PI, kEps);
  const auto obj_l = LerpObjectProto(obj_a, obj_b, 2.0);
  EXPECT_NEAR(obj_l.timestamp(), 2.0, kEps);
  EXPECT_NEAR(obj_l.pos().x(), 80.0, kEps);
  EXPECT_NEAR(obj_l.pos().y(), 100.0, kEps);
  EXPECT_NEAR(obj_l.vel().x(), vel, kEps);
  EXPECT_NEAR(obj_l.vel().y(), -2.0 * vel, kEps);
  EXPECT_NEAR(obj_l.yaw(), 0.0, kEps);
}

}  // namespace
}  // namespace qcraft::prediction
