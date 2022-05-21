#include "onboard/prediction/feature_extractor/vehicle_tnt_feature_extractor.h"

#include <cmath>

#include "gtest/gtest.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/feature_extractor/feature_extraction_util.h"
#include "onboard/prediction/feature_extractor/vehicle_tnt_feature.h"
#include "onboard/prediction/prediction_defs.h"
namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
constexpr double kEgoVelX = 2.0;
constexpr double kEgoVelY = 0.0;
constexpr double kEgoVel = 2.0;
constexpr double kEgoYaw = 0.0;
constexpr qcraft::ObjectType kEgoType = OT_VEHICLE;

constexpr double kObject1VelX = 0.0;
constexpr double kObject1VelY = 1.0;
constexpr double kObject1Vel = 1.0;
constexpr double kObject1InitPosY = 10.0;
constexpr qcraft::ObjectType kObject1Type = OT_VEHICLE;
constexpr char kObject1Id[] = "999";
constexpr double kObject1Yaw = M_PI_2;

constexpr double kUpdateTimeStep = 0.1;
constexpr double kReductionRadiusForTnt = 1.5;  // m.

ResampledObjectsHistory GenerateObjectEgoHistory(const int history_num) {
  ResampledObjectsHistory objects_history;
  objects_history.reserve(2);
  std::vector<ObjectProto> ego_history;
  ego_history.reserve(history_num);
  planner::PerceptionObjectBuilder perception_builder;
  for (int i = 0; i < history_num; ++i) {
    const double ts = kUpdateTimeStep * i;
    auto perception_obj = perception_builder.set_id(kAvObjectId)
                              .set_type(kEgoType)
                              .set_pos(Vec2d(kEgoVelX * ts, kEgoVelY * ts))
                              .set_timestamp(ts)
                              .set_yaw(kEgoYaw)
                              .set_velocity(kEgoVel)
                              .set_length_width(4.0, 2.0)
                              .set_box_center(Vec2d::Zero())
                              .Build();
    ego_history.push_back(std::move(perception_obj));
  }
  std::vector<ObjectProto> object_history;
  object_history.reserve(history_num);
  for (int i = 0; i < history_num; ++i) {
    const double ts = kUpdateTimeStep * i;
    auto perception_obj =
        perception_builder.set_id(kObject1Id)
            .set_type(kObject1Type)
            .set_pos(
                Vec2d(kObject1VelX * ts, kObject1InitPosY + kObject1VelY * ts))
            .set_timestamp(ts)
            .set_yaw(kObject1Yaw)
            .set_velocity(kObject1Vel)
            .set_length_width(4.0, 2.0)
            .set_box_center(Vec2d::Zero())
            .Build();
    object_history.push_back(std::move(perception_obj));
  }
  objects_history.push_back(ego_history);
  objects_history.push_back(object_history);
  return objects_history;
}

TEST(TntFeatureExtractorTest, ExtractVehicleTNTFeatureTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  const auto* intersection_info_ptr =
      semantic_map_manager.GetNearestIntersectionInfoAtLevel(
          semantic_map_manager.GetLevel(), Vec2d::Zero());
  constexpr int kObjectHistoryNum = 10;
  const auto objects_history = GenerateObjectEgoHistory(kObjectHistoryNum);
  TrafficLightManager::TLStateHashMap map;
  std::map<mapping::ElementId, std::vector<Vec2d>> intersection_exits_map;
  intersection_exits_map[intersection_info_ptr->id] = GetExitsOfIntersection(
      semantic_map_manager, *intersection_info_ptr, kReductionRadiusForTnt);
  const auto feature = ExtractVehicleTNTFeature(
      {objects_history[1], objects_history[0]}, semantic_map_manager,
      objects_history[0], *intersection_info_ptr, intersection_exits_map, map);
  const auto& actor_features = feature.actors_feature;

  const auto& ego_traj = actor_features.ego_traj;
  for (int i = 1; i < kObjectHistoryNum; ++i) {
    EXPECT_NEAR(ego_traj[i * kVehicleTNTConfig.coord_num],
                kEgoVelX * kUpdateTimeStep, kEps);
    EXPECT_NEAR(ego_traj[i * kVehicleTNTConfig.coord_num + 1],
                kEgoVelY * kUpdateTimeStep, kEps);
  }
  const auto& traj = actor_features.trajs;
  for (int i = 1; i < kObjectHistoryNum; ++i) {
    EXPECT_NEAR(traj[i * kVehicleTNTConfig.coord_num],
                kObject1VelX * kUpdateTimeStep, kEps);
    EXPECT_NEAR(traj[i * kVehicleTNTConfig.coord_num + 1],
                kObject1VelY * kUpdateTimeStep, kEps);
  }

  const auto& ego_speeds = actor_features.ego_speeds;
  for (int i = 0; i < kObjectHistoryNum; ++i) {
    EXPECT_NEAR(ego_speeds[i], kEgoVel, kEps);
  }

  const auto& speeds = actor_features.speeds;
  for (int i = 0; i < kObjectHistoryNum; ++i) {
    EXPECT_NEAR(speeds[i], kObject1Vel, kEps);
  }

  const auto& ego_headings = actor_features.ego_headings;
  double ego_heading_y = std::cos(kEgoYaw);
  double ego_heading_x = std::sin(kEgoYaw);
  for (int i = 0; i < kObjectHistoryNum; ++i) {
    EXPECT_NEAR(ego_headings[i * kVehicleTNTConfig.coord_num], ego_heading_x,
                kEps);
    EXPECT_NEAR(ego_headings[i * kVehicleTNTConfig.coord_num + 1],
                ego_heading_y, kEps);
  }

  double heading_y = std::cos(kObject1Yaw);
  double heading_x = std::sin(kObject1Yaw);
  const auto& headings = actor_features.headings;
  for (int i = 0; i < kObjectHistoryNum; ++i) {
    EXPECT_NEAR(headings[i * kVehicleTNTConfig.coord_num], heading_x, kEps);
    EXPECT_NEAR(headings[i * kVehicleTNTConfig.coord_num + 1], heading_y, kEps);
  }

  const auto& types = actor_features.types;
  EXPECT_EQ(types.size(), 1);
  EXPECT_EQ(types[0], kObject1Type + 1);
}
}  // namespace
}  // namespace qcraft::prediction
