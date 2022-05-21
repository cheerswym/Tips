#include "onboard/prediction/predictor/prophnet_predictor.h"

#include "gtest/gtest.h"
#include "onboard/global/clock.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft::prediction {
namespace {
std::unique_ptr<ProphNetPredictor> MakeProphNetPredictor() {
  constexpr char kProphNetParamKey[] = "prophnet_param";
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  QCHECK(param_manager != nullptr);
  NetParam prophnet_param;
  CHECK_OK(param_manager->GetProtoParam(kProphNetParamKey, &prophnet_param));
  return std::make_unique<ProphNetPredictor>(RunParamsProtoV2(),
                                             prophnet_param);
}

TEST(ProphnetPredictorTest, NormalTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const std::string id("1");
  const Vec2d pos(25.0, 42.0);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id(id)
                       .set_type(OT_VEHICLE)
                       .set_pos(pos)
                       .set_velocity(10.0)
                       .set_timestamp(0.0)
                       .set_box_center(pos)
                       .set_length_width(/*length=*/4.5, /*width=*/2.0)
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
  const auto& obj_hist = context.object_history_manager().at(id);
  const auto& av_object =
      context.av_context().GetAvObjectHistory().GetHistory();
  EXPECT_TRUE(av_object.ok());
  std::vector<const ObjectHistory*> pred_objects{&obj_hist};
  std::vector<ObjectPredictionPriority> priors{
      ObjectPredictionPriority::OPP_P0};
  TrafficLightManager::TLStateHashMap tl_state_map;
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  scenario.set_abs_dist_to_nearest_intersection(0.0);
  ProphnetPredictionInput prophnet_prediction_input = {
      av_object.value(),     pred_objects,  priors,       {scenario},
      &semantic_map_manager, &tl_state_map, &tl_state_map};
  ProphnetPredictorInput prophnet_predictor_input =
      PrepareProphnetPredictorInput(prophnet_prediction_input);
  const auto pred_results = MakeProphnetPrediction(
      prophnet_prediction_input, prophnet_predictor_input,
      *MakeProphNetPredictor().get());
  EXPECT_EQ(pred_results.size(), 1);
  EXPECT_TRUE(pred_results.find(id) != pred_results.end());
}

TEST(ProphnetPredictorTest, PrepareProphnetPredictionInputTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const std::string id("1");
  const Vec2d pos(25.0, 42.0);
  const auto obj = planner::PerceptionObjectBuilder()
                       .set_id(id)
                       .set_type(OT_VEHICLE)
                       .set_pos(pos)
                       .set_velocity(10.0)
                       .set_timestamp(0.0)
                       .set_box_center(pos)
                       .set_length_width(/*length=*/4.5, /*width=*/2.0)
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
  const auto& obj_hist = context.object_history_manager().at(id);
  const auto& av_object =
      context.av_context().GetAvObjectHistory().GetHistory();
  EXPECT_TRUE(av_object.ok());
  std::vector<const ObjectHistory*> pred_objects{&obj_hist};
  std::map<ObjectIDType, ObjectPredictionPriority> priors;
  priors[id] = ObjectPredictionPriority::OPP_P0;
  ObjectPredictionScenario scenario;
  scenario.set_road_status(ObjectRoadStatus::ORS_OFF_ROAD);
  scenario.set_intersection_status(
      ObjectIntersectionStatus::OIS_OUT_INTERSECTION);
  scenario.set_abs_dist_to_nearest_intersection(0.0);
  std::map<ObjectIDType, ObjectPredictionScenario> scenarios;
  scenarios[id] = scenario;
  const TrafficLightManager::TLStateHashMap* tl_state_map_ptr =
      &(context.traffic_light_manager().GetInferedTlStateMap());
  int max_objects_num = 64;

  const auto actual_res = PrepareProphnetPredictionInput(
      context, pred_objects, scenarios, priors, max_objects_num);

  ProphnetPredictionInput expected_res = {av_object.value(),
                                          pred_objects,
                                          {ObjectPredictionPriority::OPP_P0},
                                          {scenario},
                                          &semantic_map_manager,
                                          tl_state_map_ptr,
                                          tl_state_map_ptr};
  EXPECT_EQ(actual_res.av_obj.id(), expected_res.av_obj.id());
  EXPECT_EQ(actual_res.semantic_map_mgr, expected_res.semantic_map_mgr);
  EXPECT_EQ(actual_res.objs.size(), expected_res.objs.size());
  EXPECT_EQ(actual_res.priors.size(), expected_res.priors.size());
  EXPECT_EQ(actual_res.scenarios.size(), expected_res.scenarios.size());
  EXPECT_EQ(actual_res.inferred_tl_state_map, tl_state_map_ptr);
  EXPECT_EQ(actual_res.inferred_tl_state_map,
            &(context.traffic_light_manager().GetInferedTlStateMap()));
}

}  // namespace
}  // namespace qcraft::prediction
