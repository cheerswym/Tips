#include "onboard/planner/planner_state.h"

#include <chrono>

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gtest/gtest.h"
#include "onboard/math/test_util.h"
#include "onboard/planner/planner_util.h"

namespace qcraft::planner {
namespace {

using ::testing::Pair;
using ::testing::UnorderedElementsAre;

absl::Time MakeTime(double seconds) { return absl::FromUnixSeconds(seconds); }

TEST(PlannerState, Proto) {
  PlannerState state;
  state.planner_frame_seq_num = 9;
  state.header.set_timestamp(2);
  state.header.set_domain("planner");
  state.header.set_seq_number(10);
  state.header.set_channel("planner_state");

  {
    state.previous_trajectory_global = {{.pos = {1.0, 2.0}, .theta = 0.1},
                                        {.pos = {2.0, 3.0}, .theta = 0.2}};
    state.previous_past_trajectory_global = {
        {.pos = {-1.0, 0.0}, .theta = -0.1}, {.pos = {0.0, 1.0}, .theta = 0.0}};
    ApolloTrajectoryPointProto point;
    point.mutable_path_point()->set_x(1.0);
    point.mutable_path_point()->set_y(2.0);
    point.mutable_path_point()->set_theta(3.0);
    point.set_v(4.0);
    *state.previous_trajectory.add_trajectory_point() = point;
  }
  {
    state.stop_sign_states[0] = MakeTime(1.0);
    state.stop_sign_states[1] = MakeTime(2.0);
    state.stop_sign_yield_list[0] = {"1", "2"};
    state.stop_sign_yield_list[1] = {"3", "4"};
  }
  state.last_audio_alert_time = MakeTime(42.0);
  state.parking_brake_release_time = MakeTime(40.0);

  {
    LaneChangeStateProto lc_state;
    lc_state.set_stage(LaneChangeStage::LCS_WAITING);
    state.lane_change_state = lc_state;
  }

  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();
  {
    mapping::SemanticMapModifierProto sm_mod_proto;
    sm_mod_proto.mutable_speed_limit_modifier()->set_max_speed_limit(0.1);

    // auto *lane_id_modifier_1 =
    //     sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
    // lane_id_modifier_1->set_lane_id(69);
    // lane_id_modifier_1->set_override_speed_limit(1.0);

    // auto *lane_id_modifier_2 =
    //     sm_mod_proto.mutable_speed_limit_modifier()->add_lane_id_modifier();
    // lane_id_modifier_2->set_lane_id(4);
    // lane_id_modifier_2->set_override_speed_limit(100);

    PlannerSemanticMapModification modifier =
        CreateSemanticMapModification(semantic_map_manager, sm_mod_proto);

    state.planner_semantic_map_modifier = modifier;
  }

  state.planner_last_cycle_timeout = true;
  state.previous_autonomy_state = AutonomyStateProto();

  state.input_seq_num.set_pose(1);
  state.input_seq_num.set_autonomy_state(4);
  state.input_seq_num.set_traffic_light_states(5);
  state.input_seq_num.set_driver_action(6);
  state.input_seq_num.set_rerouting_request(7);
  state.input_seq_num.set_remote_assist_to_car(8);
  state.input_seq_num.set_recorded_route(9);
  state.input_seq_num.set_chassis(10);
  state.input_seq_num.set_prediction(11);
  state.input_seq_num.set_localization_transform(12);
  state.input_seq_num.set_routing_result(13);
  state.input_seq_num.set_av_objects(14);
  state.input_seq_num.set_real_objects(15);
  state.input_seq_num.set_virtual_objects(16);

  state.last_door_override_time = 3.0;
  state.current_time = absl::FromUnixMicros(1637127272874026L);

  PlannerState other_state = state;
  EXPECT_EQ(state, other_state);

  PlannerStateProto proto;
  state.ToProto(&proto);

  PlannerState from;
  from.FromProto(proto);

  from.planner_semantic_map_modifier = CreateSemanticMapModification(
      semantic_map_manager, proto.planner_semantic_map_modifier());

  EXPECT_EQ(from, state);
}

}  // namespace
}  // namespace qcraft::planner
