#include "gtest/gtest.h"
#include "onboard/lite/modifier/message_modifier_man.h"

namespace qcraft {

namespace {

// Returns the time at `offset` from `start`, in microseconds.
int64_t GetTimestamp(const absl::Time start, const absl::Duration offset) {
  return absl::ToUnixMicros(start + offset);
}

TEST(MessageModifierTest, RemoveObjects) {
  const std::string kObj1 = "obj_1";
  const std::string kObj2 = "obj_2";

  MessageModifierContext context;
  auto* remove_objects =
      context.execution.add_modifiers()->mutable_remove_objects_modifier();
  remove_objects->add_object_ids(kObj1);
  std::unique_ptr<MessageModifier> modifier(
      GetMessageModifier("remove_objects_modifier", context));
  ASSERT_TRUE(modifier);

  ObjectsProto objects;
  auto* object_1 = objects.add_objects();
  object_1->set_id(kObj1);
  auto* object_2 = objects.add_objects();
  object_2->set_id(kObj2);

  modifier->MaybeModify("objects_proto", &objects);

  // Only obj2 is left.
  ASSERT_EQ(1, objects.objects().size());
  EXPECT_EQ(kObj2, objects.objects(0).id());
}

TEST(MessageModifierTest, ChangeTrafficLights) {
  const int64_t kTL_1 = 100;
  const int64_t kTL_2 = 200;
  const int64_t kTL_3 = 300;

  const absl::Time kStart = absl::UniversalEpoch();
  const absl::Duration kOffset_1 = absl::Seconds(1.0);
  const absl::Duration kOffset_2 = absl::Seconds(2.0);

  MessageModifierContext context;
  context.scenario_start_time = kStart;

  // Add color modification (to RED) for the first TLs.
  auto* change_tl = context.execution.add_modifiers()
                        ->mutable_change_traffic_lights_modifier();
  {
    auto* mod = change_tl->add_mods();
    mod->set_tl_id(kTL_1);
    mod->set_target_color(TL_RED);
  }
  {
    auto* mod = change_tl->add_mods();
    mod->set_tl_id(kTL_2);
    mod->set_target_color(TL_RED);
    mod->set_start_offset_secs(absl::ToDoubleSeconds(kOffset_2));
  }

  std::unique_ptr<MessageModifier> modifier(
      GetMessageModifier("change_traffic_lights_modifier", context));
  ASSERT_TRUE(modifier);

  // Add three green TLs.
  TrafficLightStatesProto tl_states;
  tl_states.mutable_header()->set_timestamp(GetTimestamp(kStart, kOffset_1));
  {
    auto* tl_state = tl_states.add_states();
    tl_state->set_traffic_light_id(kTL_1);
    tl_state->set_color(TL_GREEN);
  }
  {
    auto* tl_state = tl_states.add_states();
    tl_state->set_traffic_light_id(kTL_2);
    tl_state->set_color(TL_GREEN);
  }
  {
    auto* tl_state = tl_states.add_states();
    tl_state->set_traffic_light_id(kTL_3);
    tl_state->set_color(TL_GREEN);
  }

  // Only TL_1 is changed to RED as TL_2 should only changed after two seconds.
  modifier->MaybeModify("traffic_light_states_proto", &tl_states);
  ASSERT_EQ(kTL_1, tl_states.states(0).traffic_light_id());
  ASSERT_EQ(kTL_2, tl_states.states(1).traffic_light_id());
  ASSERT_EQ(kTL_3, tl_states.states(2).traffic_light_id());
  EXPECT_EQ(TL_RED, tl_states.states(0).color());
  EXPECT_EQ(TL_GREEN, tl_states.states(1).color());
  EXPECT_EQ(TL_GREEN, tl_states.states(2).color());

  // At two seconds, both TL_1 and TL_2 should be changed, and TL_3 should be
  // unchanged.
  tl_states.mutable_header()->set_timestamp(GetTimestamp(kStart, kOffset_2));
  modifier->MaybeModify("traffic_light_states_proto", &tl_states);
  ASSERT_EQ(kTL_1, tl_states.states(0).traffic_light_id());
  ASSERT_EQ(kTL_2, tl_states.states(1).traffic_light_id());
  ASSERT_EQ(kTL_3, tl_states.states(2).traffic_light_id());
  EXPECT_EQ(TL_RED, tl_states.states(0).color());
  EXPECT_EQ(TL_RED, tl_states.states(1).color());
  EXPECT_EQ(TL_GREEN, tl_states.states(2).color());
}

}  // namespace

}  // namespace qcraft
