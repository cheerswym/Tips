#include "onboard/lite/modifier/message_modifier_man.h"

#include <string>

namespace qcraft {

MessageModifierMan::MessageModifierMan(const absl::Time scenario_start_time,
                                       const ScenarioExecution& execution) {
  for (const auto& modifier : execution.modifiers()) {
    MessageModifierContext context;
    context.execution = execution;
    context.scenario_start_time = scenario_start_time;
    modifiers_.emplace_back(std::make_pair(
        modifier.name(), GetMessageModifier(modifier.name(), context)));
  }
}

void MessageModifierMan::SetScenarioStartTime(
    const absl::Time scenario_start_time) {
  for (auto& [name, modifier] : modifiers_) {
    modifier->SetscenarioStartTime(scenario_start_time);
  }
}

void MessageModifierMan::Process(const std::string& channel,
                                 google::protobuf::Message* lite_msg,
                                 bool* drop_msg /*=nullptr*/) {
  bool should_drop_msg = false;
  for (const auto& name_modifier : modifiers_) {
    name_modifier.second->MaybeModify(channel, lite_msg, drop_msg);
    if (drop_msg && *drop_msg) should_drop_msg = true;
  }
  if (drop_msg && should_drop_msg) *drop_msg = should_drop_msg;
}

}  // namespace qcraft
