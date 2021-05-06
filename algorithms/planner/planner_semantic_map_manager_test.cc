#include "onboard/planner/planner_semantic_map_manager.h"

#include "gtest/gtest.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_util.h"

namespace qcraft::planner {
namespace {

TEST(PlannerSemanticMapManagerTest, Test) {
  FLAGS_planner_increase_lane_speed_limit_fraction = 0.0;

  SetMap("dojo");

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  mapping::SemanticMapModifierProto sm_mod_proto;
  PlannerSemanticMapModification modifier =
      CreateSemanticMapModification(semantic_map_manager, sm_mod_proto);
  PlannerSemanticMapManager planner_semantic_map_manager(&semantic_map_manager,
                                                         modifier);

  EXPECT_EQ(semantic_map_manager.GetLevel(),
            planner_semantic_map_manager.GetLevel());
  EXPECT_EQ(semantic_map_manager.GetLaneLengthOrDie(10),
            planner_semantic_map_manager.GetLaneLengthOrDie(10));
  EXPECT_EQ(semantic_map_manager.GetLaneControlPointsOrDie(4),
            planner_semantic_map_manager.GetLaneControlPointsOrDie(4));

  const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(69);
  const double map_speed_limit = lane_info.speed_limit;

  EXPECT_TRUE(modifier.IsEmpty());
  EXPECT_EQ(map_speed_limit,
            planner_semantic_map_manager.QueryLaneSpeedLimitById(69));
}

TEST(PlannerSemanticMapManagerTest, ModifierTest) {
  SetMap("dojo");

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  mapping::SemanticMapModifierProto sm_mod_proto;
  auto *lane_id_modifier_1 =
      sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
  lane_id_modifier_1->set_lane_id(69);
  lane_id_modifier_1->set_override_speed_limit(1.0);

  auto *lane_id_modifier_2 =
      sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
  lane_id_modifier_2->set_lane_id(4);
  lane_id_modifier_2->set_override_speed_limit(100);

  PlannerSemanticMapManager planner_semantic_map_manager(
      &semantic_map_manager,
      CreateSemanticMapModification(semantic_map_manager, sm_mod_proto));

  EXPECT_EQ(1.0, planner_semantic_map_manager.QueryLaneSpeedLimitById(69));

  const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(4);
  const double map_speed_limit = lane_info.speed_limit;

  EXPECT_EQ(map_speed_limit,
            planner_semantic_map_manager.QueryLaneSpeedLimitById(4));
}

TEST(PlannerSemanticMapManagerTest, MaxSpeedTest) {
  SetMap("dojo");

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  mapping::SemanticMapModifierProto sm_mod_proto;
  sm_mod_proto.mutable_speed_limit_modifier()->set_max_speed_limit(0.1);

  auto *lane_id_modifier_1 =
      sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
  lane_id_modifier_1->set_lane_id(69);
  lane_id_modifier_1->set_override_speed_limit(1.0);

  auto *lane_id_modifier_2 =
      sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
  lane_id_modifier_2->set_lane_id(4);
  lane_id_modifier_2->set_override_speed_limit(100);

  PlannerSemanticMapManager planner_semantic_map_manager(
      &semantic_map_manager,
      CreateSemanticMapModification(semantic_map_manager, sm_mod_proto));

  EXPECT_EQ(0.1, planner_semantic_map_manager.QueryLaneSpeedLimitById(69));

  const auto &lane_info = semantic_map_manager.FindLaneInfoOrDie(4);
  const double map_speed_limit = lane_info.speed_limit;

  EXPECT_EQ(0.1, planner_semantic_map_manager.QueryLaneSpeedLimitById(4));
  EXPECT_FALSE(map_speed_limit ==
               planner_semantic_map_manager.QueryLaneSpeedLimitById(4));
}

TEST(PlannerSemanticMapManagerTest, ProroTest) {
  SetMap("dojo");

  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  mapping::SemanticMapModifierProto sm_mod_proto;
  sm_mod_proto.mutable_speed_limit_modifier()->set_max_speed_limit(0.1);

  auto *lane_id_modifier_1 =
      sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
  lane_id_modifier_1->set_lane_id(69);
  lane_id_modifier_1->set_override_speed_limit(1.0);

  auto *lane_id_modifier_2 =
      sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
  lane_id_modifier_2->set_lane_id(4);
  lane_id_modifier_2->set_override_speed_limit(100);

  PlannerSemanticMapModification modifier_1 =
      CreateSemanticMapModification(semantic_map_manager, sm_mod_proto);
  PlannerSemanticMapManager psmm_1(&semantic_map_manager, modifier_1);

  PlannerSemanticMapModification modifier_2 = CreateSemanticMapModification(
      semantic_map_manager, PlannerSemanticMapModificationToProto(modifier_1));
  PlannerSemanticMapManager psmm_2(&semantic_map_manager, modifier_2);

  EXPECT_EQ(psmm_1.QueryLaneSpeedLimitById(69),
            psmm_2.QueryLaneSpeedLimitById(69));
  EXPECT_EQ(psmm_1.QueryLaneSpeedLimitById(4),
            psmm_2.QueryLaneSpeedLimitById(4));
}

}  // namespace
}  // namespace qcraft::planner
