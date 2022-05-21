#include "onboard/prediction/predictor/bicycle_lane_follow_predictor.h"

#include "gtest/gtest.h"
#include "onboard/global/clock.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/prediction_defs.h"
namespace qcraft::prediction {
namespace {
TEST(BicycleLaneFollowPredictorTest, OneTrajTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(-6.0, 34.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_CYCLIST)
                 .set_pos(pos)
                 .set_velocity(5.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(-M_PI * 0.5)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  PredictionContext context;
  TrafficLightStatesProto tl_states;
  VehicleGeometryParamsProto params;
  PoseProto av_pose;
  Vec3dProto av_pos;
  Vec3dToProto(Vec3d(0.0, 0.0, 0.0), &av_pos);
  *av_pose.mutable_pos_smooth() = av_pos;
  auto loc_transform = std::make_shared<LocalizationTransformProto>();
  context.UpdateLocalizationTransform(loc_transform);
  QLOG_IF_NOT_OK(WARNING, context.Update(Clock::Now(), semantic_map_manager,
                                         tl_states, av_pose, params));
  context.UpdateObjects(objects_proto, *loc_transform, /*thread_pool=*/nullptr);
  const auto& hist = context.object_history_manager().at(id);
  const auto trajs = MakeBicycleLaneFollowPrediction(
      &hist, context, ObjectPredictionPriority::OPP_P2);
  EXPECT_EQ(trajs.size(), 1);
}

TEST(BicycleLaneFollowPredictorTest, TwoTrajsTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(-6.0, 34.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_CYCLIST)
                 .set_pos(pos)
                 .set_velocity(5.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(-M_PI * 0.5 + M_PI / 6.0)
                 .Build();
  ObjectsProto objects_proto;
  *objects_proto.add_objects() = std::move(obj);
  PredictionContext context;
  TrafficLightStatesProto tl_states;
  VehicleGeometryParamsProto params;
  PoseProto av_pose;
  Vec3dProto av_pos;
  Vec3dToProto(Vec3d(0.0, 0.0, 0.0), &av_pos);
  *av_pose.mutable_pos_smooth() = av_pos;
  auto loc_transform = std::make_shared<LocalizationTransformProto>();
  context.UpdateLocalizationTransform(loc_transform);
  QLOG_IF_NOT_OK(WARNING, context.Update(Clock::Now(), semantic_map_manager,
                                         tl_states, av_pose, params));
  context.UpdateObjects(objects_proto, *loc_transform, /*thread_pool=*/nullptr);
  const auto& hist = context.object_history_manager().at(id);
  const auto trajs = MakeBicycleLaneFollowPrediction(
      &hist, context, ObjectPredictionPriority::OPP_P2);
  EXPECT_EQ(trajs.size(), 2);
}

}  // namespace
}  // namespace qcraft::prediction
