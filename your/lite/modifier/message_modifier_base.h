#ifndef ONBOARD_LITE_MODIFIER_MESSAGE_MODIFIER_BASE_H_
#define ONBOARD_LITE_MODIFIER_MESSAGE_MODIFIER_BASE_H_

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "offboard/simulation/proto/scenario.pb.h"
#include "onboard/global/registry.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {

// Registers a message modifier with the `class_name` and `alias`.
#define REGISTER_MESSAGE_MODIFIER(class_name, alias)         \
  REGISTER_SUBCLASS_NAME(MessageModifier, class_name, alias, \
                         const MessageModifierContext&);     \
  REGISTER_SUBCLASS(MessageModifier, class_name, const MessageModifierContext&)

struct MessageModifierContext {
  ScenarioExecution execution;
  absl::Time scenario_start_time;
};

template <class C>
std::optional<C> GetModifierProto(const ScenarioExecution& execution) {
  const C c;
  const auto modifier_name =
      ConvertToLowerUnderscore(c.GetDescriptor()->name());
  for (const auto& modifier : execution.modifiers()) {
    const auto* descriptor = modifier.GetDescriptor();
    if (descriptor == nullptr) continue;

    const auto* modifier_desc = descriptor->FindFieldByName(modifier_name);
    if (modifier_desc == nullptr) continue;

    if (!modifier.GetReflection()->HasField(modifier, modifier_desc)) continue;

    return static_cast<const C&>(
        modifier.GetReflection()->GetMessage(modifier, modifier_desc));
  }

  return std::nullopt;
}

// Base class for message modifier which can be used to modify any given lite
// messages.
class MessageModifier {
 public:
  explicit MessageModifier(const MessageModifierContext& context)
      : context_(context) {}
  virtual ~MessageModifier() {}

  virtual void MaybeModify(const std::string& channel,
                           google::protobuf::Message* lite_msg,
                           bool* drop_msg = nullptr) = 0;

  // Returns the time elapsed relative to the scenario start.
  absl::Duration GetOffset(const google::protobuf::Message& lite_msg);

  // Sets the scenario start time.
  void SetscenarioStartTime(const absl::Time scenario_start_time) {
    context_.scenario_start_time = scenario_start_time;
  }

 private:
  MessageModifierContext context_;
};

// Creates a message modifier of the given `modifier_name_or_alias`, which must
// be valid. Returns the pointer to the created modifier.
MessageModifier* GetMessageModifier(const std::string& modifier_name_or_alias,
                                    const MessageModifierContext& context);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_MESSAGE_MODIFIER_BASE_H_
