#include "onboard/prediction/scheduler/priority_analyzer.h"

#include "gtest/gtest.h"
#include "onboard/global/clock.h"
#include "onboard/planner/test_util/perception_object_builder.h"
namespace qcraft::prediction {
namespace {
TEST(PriorityAnalyzerTest, IgnoreTest1) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(0.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  auto loc_transform = std::make_shared<LocalizationTransformProto>();
  *av_pose.mutable_pos_smooth() = av_pos;
  context.UpdateLocalizationTransform(loc_transform);
  QLOG_IF_NOT_OK(WARNING, context.Update(Clock::Now(), semantic_map_manager,
                                         tl_states, av_pose, params));
  context.UpdateObjects(objects_proto, *loc_transform, /*thread_pool=*/nullptr);
  const auto& hist = context.object_history_manager().at(id);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P3);
}

TEST(PriorityAnalyzerTest, IgnoreTest2) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  auto loc_transform = std::make_shared<LocalizationTransformProto>();
  *av_pose.mutable_pos_smooth() = av_pos;
  context.UpdateLocalizationTransform(loc_transform);
  QLOG_IF_NOT_OK(WARNING, context.Update(Clock::Now(), semantic_map_manager,
                                         tl_states, av_pose, params));
  context.UpdateObjects(objects_proto, *loc_transform, /*thread_pool=*/nullptr);
  const auto& hist = context.object_history_manager().at(id);
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P3);
}

TEST(PriorityAnalyzerTest, ScanBoxTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(13.0, -6.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P2);
}

TEST(PriorityAnalyzerTest, OnRoadTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_ON_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P2);
}

TEST(PriorityAnalyzerTest, InIntersectionTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(13.0, -31.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_IN_INTERSECTION);

  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P2);
}

TEST(PriorityAnalyzerTest, PedTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(100.0, 0.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_CYCLIST)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_abs_dist_to_nearest_lane(1.0);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P2);
}

TEST(PriorityAnalyzerTest, NoPedTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const Vec2d pos(100.0, 0.0);
  std::string id("1");
  auto obj = planner::PerceptionObjectBuilder()
                 .set_id(id)
                 .set_type(OT_VEHICLE)
                 .set_pos(pos)
                 .set_velocity(1.0)
                 .set_timestamp(0.0)
                 .set_box_center(pos)
                 .set_length_width(/*length=*/2.0, /*width=*/1.0)
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
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_abs_dist_to_nearest_lane(1.0);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const auto priorities = AnalyzePriorities(context, {&hist}, scenarios);
  EXPECT_EQ(priorities.at(id), ObjectPredictionPriority::OPP_P3);
}

}  // namespace
}  // namespace qcraft::prediction
