#ifndef ONBOARD_LITE_MODIFIER_CHANGE_TRAFFIC_LIGHTS_H_
#define ONBOARD_LITE_MODIFIER_CHANGE_TRAFFIC_LIGHTS_H_

#include <string>
#include <vector>

#include "onboard/lite/modifier/message_modifier_base.h"

namespace qcraft {

// A message modifier that changes traffic light states.
class ChangeTrafficLights : public MessageModifier {
 public:
  explicit ChangeTrafficLights(const MessageModifierContext& context)
      : MessageModifier(context) {
    const auto modifier_or =
        GetModifierProto<ChangeTrafficLightsModifier>(context.execution);
    if (modifier_or.has_value()) {
      for (const auto& mod : modifier_or->mods()) {
        mods_.push_back(mod);
      }
    }
  }

  void MaybeModify(const std::string& channel,
                   google::protobuf::Message* lite_msg,
                   bool* drop_msg = nullptr) override {
    if (mods_.empty() || channel != "traffic_light_states_proto") {
      return;
    }

    auto* tl_states = static_cast<TrafficLightStatesProto*>(lite_msg);

    for (auto& state : *tl_states->mutable_states()) {
      for (const auto& mod : mods_) {
        if (mod.tl_id() != state.traffic_light_id()) continue;

        // Skip if we are out of trigger period.
        const auto& start_offset = absl::Seconds(mod.start_offset_secs());
        const auto& end_offset = mod.has_end_offset_secs()
                                     ? absl::Seconds(mod.end_offset_secs())
                                     : absl::InfiniteDuration();
        const auto& cur_offset = GetOffset(*lite_msg);
        if ((cur_offset < start_offset) || (cur_offset > end_offset)) continue;

        state.set_color(mod.target_color());
        state.set_flashing(mod.target_flashing());
      }
    }
  }

 private:
  // A map from traffic light id to desired modification.
  std::vector<ChangeTrafficLightsModifier::Mod> mods_;
};
REGISTER_MESSAGE_MODIFIER(ChangeTrafficLights, change_traffic_lights_modifier);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_CHANGE_TRAFFIC_LIGHTS_H_
