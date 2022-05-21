#ifndef ONBOARD_LITE_MODIFIER_MESSAGE_MODIFIER_MAN_H_
#define ONBOARD_LITE_MODIFIER_MESSAGE_MODIFIER_MAN_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "offboard/simulation/proto/scenario.pb.h"
#include "onboard/global/registry.h"
#include "onboard/lite/modifier/change_traffic_lights.h"
#include "onboard/lite/modifier/keep_objects.h"
#include "onboard/lite/modifier/message_modifier_base.h"
#include "onboard/lite/modifier/remove_objects.h"
#include "onboard/lite/modifier/replace_objects.h"
#include "onboard/lite/modifier/teleport_sdc.h"

namespace qcraft {

// A class that manages all message modifiers. It is used to initialize and
// invoke specified modifiers.
class MessageModifierMan {
 public:
  explicit MessageModifierMan(const absl::Time scenario_start_time,
                              const ScenarioExecution& execution);

  void Process(const std::string& channel, google::protobuf::Message* lite_msg,
               bool* drop_msg = nullptr);

  void SetScenarioStartTime(const absl::Time scenario_start_time);

 private:
  std::vector<std::pair<std::string, std::unique_ptr<MessageModifier>>>
      modifiers_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_MESSAGE_MODIFIER_MAN_H_
