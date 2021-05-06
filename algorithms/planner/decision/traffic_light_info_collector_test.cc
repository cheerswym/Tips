#include "onboard/planner/decision/traffic_light_info_collector.h"

#include <vector>

#include "gtest/gtest.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {
namespace {

constexpr double kEpsilon = 0.1;  // m

// Build traffic light states proto.
TrafficLightStatesProto BuildTrafficLightStatesProto(
    const std::vector<int> &traffic_light_ids,
    const std::vector<bool> &traffic_light_flashings,
    const std::vector<TrafficLightColor> &traffic_light_colors) {
  QCHECK_EQ(traffic_light_ids.size(), traffic_light_flashings.size());
  QCHECK_EQ(traffic_light_ids.size(), traffic_light_colors.size());

  TrafficLightStatesProto traffic_light_states;
  const int size = traffic_light_ids.size();
  for (int i = 0; i < size; i++) {
    auto *tl_state = traffic_light_states.add_states();
    tl_state->set_traffic_light_id(traffic_light_ids[i]);
    tl_state->set_flashing(traffic_light_flashings[i]);
    tl_state->set_color(traffic_light_colors[i]);
  }

  return traffic_light_states;
}

// StarPointControlTest. Av before stop line.
TEST(TLInfoCollectorTest, StartPointControl_BeforeStopLine_Test) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  vantage_client_man::CreateVantageClientMan(*param_manager);

  // Load planner semantic map.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct route sections.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(0.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  SendRouteSectionsAreaToCanvas(&smm, route_sections,
                                "startPointControl/straight/before_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{891, 889},
      /*traffic_light_flahings=*/{false, false}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_GREEN, TrafficLightColor::TL_RED});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));

  EXPECT_EQ(output.tl_info_map.size(), 3);
  for (const auto &[id, tl_info] : output.tl_info_map) {
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 68.846, kEpsilon);
    EXPECT_FALSE(tl_info.can_go_on_red());

    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
    EXPECT_EQ(iter->second.tl_id, 889);
  }
}

// StarPointControlTest. Av has passed stop line.
TEST(TLInfoCollectorTest, StartPointControl_PassedStopLine_Test) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  vantage_client_man::CreateVantageClientMan(*param_manager);

  // Load planner semantic map.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct route sections.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/10.0, Vec2d(87.267, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "b7_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  SendRouteSectionsAreaToCanvas(&smm, route_sections,
                                "startPointControl/straight/passed_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{891, 889},
      /*traffic_light_flahings=*/{false, false}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_GREEN, TrafficLightColor::TL_RED});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));

  EXPECT_EQ(output.tl_info_map.size(), 3);
  for (const auto &[id, tl_info] : output.tl_info_map) {
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), -18.29, kEpsilon);
    EXPECT_FALSE(tl_info.can_go_on_red());

    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
    EXPECT_EQ(iter->second.tl_id, 889);
  }
}

// TODO(jiayu): Check green flashing traffic light turn red left time.
// MultiControlPointTest. Av before first stop line.
TEST(TLInfoCollectorTest, MultiControlPoint_BeforeFirstStopLine_Test) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  vantage_client_man::CreateVantageClientMan(*param_manager);

  // Load planner semantic map.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(561.585, -461.978), 0.0, Vec2d(11.0, 0.0));
  const auto route_path =
      RoutingToLanePoint(smm, sdc_pose, mapping::LanePoint(6613, 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, *route_path);
  SendRouteSectionsAreaToCanvas(
      &smm, route_sections,
      "MultiControlPoint/turn_left/before_first_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{6846, 6845},
      /*traffic_light_flahings=*/{false, true}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_RED, TrafficLightColor::TL_GREEN});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  const auto &tl_info_map = output.tl_info_map;

  EXPECT_EQ(tl_info_map.size(), 4);
  // Go straight lane 6825.
  {
    const auto iter_tl_info = tl_info_map.find(6825);
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto &tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::LEFT_WAITING_AREA);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 49.308, kEpsilon);
    EXPECT_NEAR(tl_info.control_point_relative_s().back(), 60.907, kEpsilon);
    const auto iter_straight =
        tl_info.tls().find(TrafficLightDirection::STRAIGHT);
    EXPECT_EQ(iter_straight->second.tl_id, 6846);
    EXPECT_EQ(iter_straight->second.tl_state, TrafficLightState::TL_STATE_RED);

    const auto iter_left = tl_info.tls().find(TrafficLightDirection::LEFT);
    EXPECT_EQ(iter_left->second.tl_id, 6845);
    EXPECT_EQ(iter_left->second.tl_state,
              TrafficLightState::TL_STATE_GREEN_FLASHING);
  }

  // Go straight lane 6816.
  {
    const auto iter_tl_info = tl_info_map.find(6816);
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto &tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 49.308, kEpsilon);
    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_id, 6846);
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
  }

  // UTurn lane 6830.
  {
    const auto iter_tl_info = tl_info_map.find(6830);
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto &tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 49.308, kEpsilon);
    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_id, 6845);
    EXPECT_EQ(iter->second.tl_state,
              TrafficLightState::TL_STATE_GREEN_FLASHING);
  }

  // Go Straight lane 6817.
  {
    const auto iter_tl_info = tl_info_map.find(6817);
    EXPECT_TRUE(iter_tl_info != tl_info_map.end());
    const auto &tl_info = iter_tl_info->second;
    EXPECT_EQ(tl_info.tl_control_type(),
              TrafficLightControlType::SINGLE_DIRECTION);
    EXPECT_FALSE(tl_info.can_go_on_red());
    EXPECT_NEAR(tl_info.control_point_relative_s().front(), 49.308, kEpsilon);
    const auto iter = tl_info.tls().find(TrafficLightDirection::UNMARKED);
    EXPECT_EQ(iter->second.tl_id, 6846);
    EXPECT_EQ(iter->second.tl_state, TrafficLightState::TL_STATE_RED);
  }
}

TEST(TLInfoCollectorTest, EmptyTest) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  vantage_client_man::CreateVantageClientMan(*param_manager);

  // Load planner semantic map.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(412.277, 209.654), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToNameSpot(smm, sdc_pose, "hw_e2_end");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, route_path);
  SendRouteSectionsAreaToCanvas(&smm, route_sections, "EmptyTest");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{},
      /*traffic_light_flahings=*/{}, /*traffic_light_colors=*/
      {});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  EXPECT_TRUE(output.tl_info_map.empty());
}

// MultiControlPointTest. AV before second stop line, has passed first stop
// line.
TEST(TLInfoCollectorTest, MultiControlPoint_BeforeSecondStopline_Test) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  vantage_client_man::CreateVantageClientMan(*param_manager);

  // Load planner semantic map.
  SetMap("dojo");
  SemanticMapManager smm;
  smm.LoadWholeMap().Build();
  PlannerSemanticMapModification psmm_modifier;
  PlannerSemanticMapManager psmm(&smm, psmm_modifier);

  // Construct route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(618.370, -461.186), 0.0, Vec2d(11.0, 0.0));
  const auto route_path =
      RoutingToLanePoint(smm, sdc_pose, mapping::LanePoint(6613, 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(smm, *route_path);
  SendRouteSectionsAreaToCanvas(
      &smm, route_sections,
      "MultiControlPoint/turn_left/before_second_stop_line");

  // Build traffic light states proto.
  const auto traffic_light_states = BuildTrafficLightStatesProto(
      /*traffic_light_ids=*/{6846, 6845},
      /*traffic_light_flahings=*/{false, false}, /*traffic_light_colors=*/
      {TrafficLightColor::TL_RED, TrafficLightColor::TL_GREEN});

  YellowLightObservationsNew yellow_light_observations;
  ASSIGN_OR_DIE(const auto output,
                CollectTrafficLightInfo(
                    TrafficLightInfoCollectorInput{
                        .psmm = &psmm,
                        .traffic_light_states = &traffic_light_states,
                        .route_sections = &route_sections},
                    yellow_light_observations));
  const auto &tl_info_map = output.tl_info_map;

  EXPECT_EQ(tl_info_map.size(), 1);
  const auto iter_tl_info = tl_info_map.find(6825);
  EXPECT_TRUE(iter_tl_info != tl_info_map.end());
  const auto &tl_info = iter_tl_info->second;
  EXPECT_EQ(tl_info.tl_control_type(),
            TrafficLightControlType::LEFT_WAITING_AREA);
  EXPECT_FALSE(tl_info.can_go_on_red());
  EXPECT_NEAR(tl_info.control_point_relative_s().front(), -7.525, kEpsilon);
  EXPECT_NEAR(tl_info.control_point_relative_s().back(), 4.073, kEpsilon);
  const auto iter_straight =
      tl_info.tls().find(TrafficLightDirection::STRAIGHT);
  EXPECT_EQ(iter_straight->second.tl_id, 6846);
  EXPECT_EQ(iter_straight->second.tl_state, TrafficLightColor::TL_RED);

  const auto iter_left = tl_info.tls().find(TrafficLightDirection::LEFT);
  EXPECT_EQ(iter_left->second.tl_id, 6845);
  EXPECT_EQ(iter_left->second.tl_state, TrafficLightState::TL_STATE_GREEN);
}

}  // namespace
}  // namespace qcraft::planner
