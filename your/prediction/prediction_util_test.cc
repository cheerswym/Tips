#include "onboard/prediction/prediction_util.h"

#include "absl/strings/str_split.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace prediction {
namespace {

using planner::PerceptionObjectBuilder;
using testing::UnorderedElementsAre;
TEST(AlignPerceptionObjectTime, Works) {
  auto perception_object = PerceptionObjectBuilder()
                               .set_id("obj")
                               .set_pos(Vec2d(1.0, 2.0))
                               .set_timestamp(10.0)
                               .set_length_width(4.0, 2.0)
                               .set_yaw(0.0)
                               .set_yaw_rate(0.1)
                               .set_speed(Vec2d(1.0, 2.0))
                               .set_accel(Vec2d(1.0, -1.0))
                               .Build();
  ObjectProto& aligned_object = perception_object;
  ASSERT_OK(AlignPerceptionObjectTime(10.1, &aligned_object));
  EXPECT_THAT(aligned_object, ProtoPartialApproximatelyMatchesText(R"(
                                      pos { x: 1.105 y: 2.195 }
                                      vel { x: 1.1 y: 1.9 }
                                      accel { x: 1 y: -1 }
                                      timestamp: 10.1
                                      id: "obj"
                                      yaw: 0.01
                                      yaw_rate: 0.1
                                      bounding_box {
                                        x: 1.105
                                        y: 2.195
                                        heading: 0.01
                                        width: 2
                                        length: 4
                                      }
                                      bounding_box_source: FIERY_EYE_NET
                                      parked: false
                                      offroad: false)",
                                                                   1e-3));
  const Box2d box(aligned_object.bounding_box());
  const Polygon2d shifted_polygon =
      Polygon2d::FromPoints(aligned_object.contour(), /*is_convex=*/true)
          .value();
  EXPECT_THAT(
      shifted_polygon.points(),
      UnorderedElementsAre(Vec2dNear(box.GetCorner(Box2d::FRONT_LEFT), 1e-3),
                           Vec2dNear(box.GetCorner(Box2d::REAR_LEFT), 1e-3),
                           Vec2dNear(box.GetCorner(Box2d::REAR_RIGHT), 1e-3),
                           Vec2dNear(box.GetCorner(Box2d::FRONT_RIGHT), 1e-3)));
}

TEST(PerceptionUtil, BuildObjectBoundingBox) {
  ObjectProto object;
  TextToProto(R"(
        type: OT_VEHICLE
        id:  "abc"
        pos { x: 1 y: 1.5 }
        vel { x: 0 y: 2 }
        accel { x: 0 y: 1 }
        contour { x: 0.0 y: 4.0 }
        contour { x: 0.0 y: 0.0 }
        contour { x: 1 y: 0 }
        contour { x: 1 y: 4 }
        timestamp: 10
        yaw: 1.571
        yaw_rate: 0.2
        parked: false
        offroad: false
        moving_state: MS_MOVING
        min_z: 0
        max_z: 2.2
        bounding_box { x: 1 y: 1.5 heading: 1.571 width: 2 length: 5 }
        )",
              &object);
  auto box = BuildObjectBoundingBox(object);
  EXPECT_EQ(box.length(), 5.0);
  EXPECT_EQ(box.width(), 2.0);

  object.clear_bounding_box();
  ASSERT_FALSE(object.has_bounding_box());

  box = BuildObjectBoundingBox(object);
  EXPECT_NEAR(box.length(), 4.0, 1e-3);
  EXPECT_NEAR(box.width(), 1.0, 1e-3);
}

TEST(FindTrajCurbCollisionIndex, FindLaneBoundaryByIdTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  const auto level_id = semantic_map_manager.GetLevel();
  std::string boundary_info;
  Segment2d nda_segment;
  const Vec2d check_pos(14.0, 4.0);
  ASSERT_TRUE(semantic_map_manager.GetNearestNamedImpassableBoundaryAtLevel(
      level_id, check_pos, &nda_segment, &boundary_info));

  std::vector<std::string> boundary_info_ = absl::StrSplit(boundary_info, "|");
  ASSERT_GE(boundary_info_.size(), 2);
  EXPECT_EQ(boundary_info_[0], "CURB");
  EXPECT_EQ(boundary_info_[1], "382");

  const auto* lane_boundary_proto =
      semantic_map_manager
          .FindLaneBoundaryByIdOrDie(std::stoi(boundary_info_[1]))
          .proto;
  EXPECT_EQ(lane_boundary_proto->type(),
            mapping::LaneBoundaryProto_Type::LaneBoundaryProto_Type_CURB);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft
