#include "onboard/prediction/predictor/heuristic_pedestrian_predictor.h"

#include "gtest/gtest.h"
#include "onboard/global/clock.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/prediction_defs.h"
namespace qcraft::prediction {
namespace {
TEST(HeuristicPedestrianPredictorTest, OneStateTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(8.0, 19.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_PEDESTRIAN)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
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
  const auto traj = MakeHeuristicPedestrianPrediction(
      &hist, semantic_map_manager, ObjectPredictionPriority::OPP_P2);
  EXPECT_EQ(traj.type(), PT_CYCV);
}

TEST(HeuristicPedestrianPredictorTest, NormalTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(15.0, 38.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_PEDESTRIAN)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
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
  const Vec2d pos2(14.9, 38.0);
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos2)
                  .set_velocity(1.0)
                  .set_timestamp(0.1)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  ObjectsProto objects_proto2;
  *objects_proto2.add_objects() = std::move(obj2);
  context.UpdateObjects(objects_proto2, *loc_transform,
                        /*thread_pool=*/nullptr);

  const auto& hist = context.object_history_manager().at(id);
  const auto traj = MakeHeuristicPedestrianPrediction(
      &hist, semantic_map_manager, ObjectPredictionPriority::OPP_P2);
  EXPECT_EQ(traj.type(), PT_PED_KINEMATIC);
  EXPECT_EQ(traj.points().size(),
            static_cast<int>(kPredictionDuration / kPredictionTimeStep));
}

TEST(HeuristicPedestrianPredictorTest, CurbCutoffTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(-27.0, 23.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_PEDESTRIAN)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/0.5, /*width=*/0.5)
                 .set_yaw(0.0)
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
  const Vec2d pos2(-26.9, 23.0);
  auto obj2 = planner::PerceptionObjectBuilder()
                  .set_id(id)
                  .set_type(OT_PEDESTRIAN)
                  .set_pos(pos2)
                  .set_velocity(1.0)
                  .set_timestamp(0.1)
                  .set_box_center(pos2)
                  .set_length_width(/*length=*/0.5, /*width=*/0.5)
                  .set_yaw(0.0)
                  .Build();
  ObjectsProto objects_proto2;
  *objects_proto2.add_objects() = std::move(obj2);
  context.UpdateObjects(objects_proto2, *loc_transform,
                        /*thread_pool=*/nullptr);

  const auto& hist = context.object_history_manager().at(id);
  const auto traj = MakeHeuristicPedestrianPrediction(
      &hist, semantic_map_manager, ObjectPredictionPriority::OPP_P2);
  EXPECT_EQ(traj.type(), PT_PED_KINEMATIC);
  EXPECT_EQ(traj.points().size(),
            static_cast<int>(kSafeHorizon / kPredictionTimeStep));
}

}  // namespace
}  // namespace qcraft::prediction
