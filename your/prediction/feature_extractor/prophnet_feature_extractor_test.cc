#include "onboard/prediction/feature_extractor/prophnet_feature_extractor.h"

#include "gtest/gtest.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"

namespace qcraft::prediction {
namespace {
constexpr double kEps = 1e-5;
constexpr int kDefaultHistoryNum = 11;
using prophnet::kCoords;
using prophnet::kHistoryNum;
ResampledObjectsHistory GenerateObjectEgoHistory(const int history_num) {
  constexpr double kUpdateTimeStep = 0.1;
  ResampledObjectsHistory objects_history;
  objects_history.reserve(2);
  std::vector<ObjectProto> object_history;
  std::vector<ObjectProto> ego_history;
  object_history.reserve(history_num);
  ego_history.reserve(history_num);
  planner::PerceptionObjectBuilder perception_builder;
  for (int i = 0; i < history_num; ++i) {
    const std::string object_id = "999";
    const double vel = 2.0;
    const double ts = kUpdateTimeStep * i;
    const auto perception_obj = perception_builder.set_id(object_id)
                                    .set_type(OT_VEHICLE)
                                    .set_pos(Vec2d(vel * ts, 0.0))
                                    .set_timestamp(ts)
                                    .set_yaw(0.0)
                                    .set_velocity(vel)
                                    .set_length_width(4.0, 2.0)
                                    .set_box_center(Vec2d::Zero())
                                    .Build();
    object_history.push_back(perception_obj);
  }
  for (int i = 0; i < kDefaultHistoryNum; ++i) {
    const double vel = 1.0;
    const double ts = kUpdateTimeStep * i;
    const auto ego_obj = perception_builder.set_id(kAvObjectId)
                             .set_type(OT_VEHICLE)
                             .set_pos(Vec2d(0.0, 10.0 + vel * ts))
                             .set_timestamp(ts)
                             .set_yaw(M_PI_2)
                             .set_velocity(vel)
                             .set_length_width(4.0, 2.0)
                             .set_box_center(Vec2d::Zero())
                             .Build();
    ego_history.push_back(ego_obj);
  }
  objects_history.push_back(object_history);
  objects_history.push_back(ego_history);
  return objects_history;
}

TEST(ProphnetFeatureExtractorTest, ExtractLessThanOneSecondTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  TrafficLightManager::TLStateHashMap tl_states;
  semantic_map_manager.LoadWholeMap().Build();
  constexpr int kObjectHistoryNum = 5;
  const auto objects_history = GenerateObjectEgoHistory(kObjectHistoryNum);
  const auto feature =
      ExtractProphnetFeature(objects_history, tl_states, &semantic_map_manager);
  const auto& actor_features = feature.actors_feature;
  const auto& ego_pos = actor_features.ego_pos;
  EXPECT_NEAR(ego_pos[0], 0.0, kEps);
  EXPECT_NEAR(ego_pos[1], 11.0, kEps);

  const auto& rot_mat = actor_features.rot_mat;
  EXPECT_NEAR(rot_mat[0], 0.0, kEps);
  EXPECT_NEAR(rot_mat[1], 1.0, kEps);
  EXPECT_NEAR(rot_mat[2], -1.0, kEps);
  EXPECT_NEAR(rot_mat[3], 0.0, kEps);

  const auto& trajs = actor_features.trajs;
  for (int i = 0; i < kHistoryNum - kObjectHistoryNum; ++i) {
    EXPECT_NEAR(trajs[i * kCoords], 0.0, kEps);
    EXPECT_NEAR(trajs[i * kCoords + 1], 0.0, kEps);
  }
  EXPECT_NEAR(trajs[18], 0.0, kEps);
  EXPECT_NEAR(trajs[19], -0.2, kEps);
  for (int i = kHistoryNum; i < 2 * kHistoryNum; ++i) {
    EXPECT_NEAR(trajs[i * kCoords], 0.1, kEps);
    EXPECT_NEAR(trajs[i * kCoords + 1], 0.0, kEps);
  }

  const auto& speeds = actor_features.speeds;
  for (int i = 0; i < kHistoryNum - kObjectHistoryNum; ++i) {
    EXPECT_NEAR(speeds[i], 0.0, kEps);
  }
  EXPECT_NEAR(speeds[9], 2.0, kEps);
  for (int i = kHistoryNum; i < 2 * kHistoryNum; ++i) {
    EXPECT_NEAR(speeds[i], 1.0, kEps);
  }

  const auto& headings = actor_features.headings;
  for (int i = 0; i < kHistoryNum - kObjectHistoryNum; ++i) {
    EXPECT_NEAR(headings[i * kCoords], 0.0, kEps);
    EXPECT_NEAR(headings[i * kCoords + 1], 0.0, kEps);
  }
  EXPECT_NEAR(headings[18], -1.0, kEps);
  EXPECT_NEAR(headings[19], 0.0, kEps);
  for (int i = kHistoryNum; i < 2 * kHistoryNum; ++i) {
    EXPECT_NEAR(headings[i * kCoords], 0.0, kEps);
    EXPECT_NEAR(headings[i * kCoords + 1], 1.0, kEps);
  }

  const auto& types = actor_features.types;
  EXPECT_EQ(types.size(), 2);
  EXPECT_NEAR(types[0], 2.0, kEps);
  EXPECT_NEAR(types[1], 2.0, kEps);

  const auto& cur_poses = actor_features.cur_poses;
  EXPECT_NEAR(cur_poses[0], -11.0, kEps);
  EXPECT_NEAR(cur_poses[1], -0.8, kEps);
  EXPECT_NEAR(cur_poses[2], 0.0, kEps);
  EXPECT_NEAR(cur_poses[3], 0.0, kEps);

  const auto& shapes = actor_features.shapes;
  EXPECT_NEAR(shapes[0], 4.0, kEps);
  EXPECT_NEAR(shapes[1], 2.0, kEps);

  const auto& max_ks = actor_features.max_ks;
  EXPECT_NEAR(max_ks[0], kLengthToCurvaturePlf(4.0), kEps);

  const auto& max_lat_accs = actor_features.max_lat_accs;
  EXPECT_NEAR(max_lat_accs[0], kLengthToMaxLatAccPlf(4.0), kEps);

  const auto& dist_to_racs = actor_features.dist_to_racs;
  EXPECT_NEAR(dist_to_racs[0], 0.5 * std::min(kLengthToWheelbasePlf(4.0), 4.0),
              kEps);
}

TEST(ProphnetFeatureExtractorTest, ExtractMoreThanOneSecondTest) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  TrafficLightManager::TLStateHashMap tl_states;
  const auto objects_history = GenerateObjectEgoHistory(kDefaultHistoryNum);
  const auto feature =
      ExtractProphnetFeature(objects_history, tl_states, &semantic_map_manager);
  const auto& actor_features = feature.actors_feature;
  const auto& ego_pos = actor_features.ego_pos;
  EXPECT_NEAR(ego_pos[0], 0.0, kEps);
  EXPECT_NEAR(ego_pos[1], 11.0, kEps);

  const auto& rot_mat = actor_features.rot_mat;
  EXPECT_NEAR(rot_mat[0], 0.0, kEps);
  EXPECT_NEAR(rot_mat[1], 1.0, kEps);

  const auto& trajs = actor_features.trajs;
  for (int i = 0; i < kHistoryNum; ++i) {
    EXPECT_NEAR(trajs[i * kCoords], 0.0, kEps);
    EXPECT_NEAR(trajs[i * kCoords + 1], -0.2, kEps);
  }
  for (int i = kHistoryNum; i < 2 * kHistoryNum; ++i) {
    EXPECT_NEAR(trajs[i * kCoords], 0.1, kEps);
    EXPECT_NEAR(trajs[i * kCoords + 1], 0.0, kEps);
  }

  const auto& speeds = actor_features.speeds;
  for (int i = 0; i < kHistoryNum; ++i) {
    EXPECT_NEAR(speeds[i], 2.0, kEps);
  }
  for (int i = kHistoryNum; i < 2 * kHistoryNum; ++i) {
    EXPECT_NEAR(speeds[i], 1.0, kEps);
  }

  const auto& headings = actor_features.headings;
  for (int i = 0; i < kHistoryNum; ++i) {
    EXPECT_NEAR(headings[i * kCoords], -1.0, kEps);
    EXPECT_NEAR(headings[i * kCoords + 1], 0.0, kEps);
  }
  for (int i = kHistoryNum; i < 2 * kHistoryNum; ++i) {
    EXPECT_NEAR(headings[i * kCoords], 0.0, kEps);
    EXPECT_NEAR(headings[i * kCoords + 1], 1.0, kEps);
  }

  const auto& types = actor_features.types;
  EXPECT_EQ(types.size(), 2);
  EXPECT_NEAR(types[0], 2.0, kEps);
  EXPECT_NEAR(types[1], 2.0, kEps);

  const auto& cur_poses = actor_features.cur_poses;
  EXPECT_NEAR(cur_poses[0], -11.0, kEps);
  EXPECT_NEAR(cur_poses[1], -2.0, kEps);
  EXPECT_NEAR(cur_poses[2], 0.0, kEps);
  EXPECT_NEAR(cur_poses[3], 0.0, kEps);
}

}  // namespace
}  // namespace qcraft::prediction
