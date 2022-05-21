#include "onboard/lite/modifier/message_modifier_base.h"

#include <string>

#include "onboard/global/registry.h"
#include "onboard/lite/lite_message_util.h"

namespace qcraft {

MessageModifier* GetMessageModifier(const std::string& modifier_name_or_alias,
                                    const MessageModifierContext& context) {
  return Registry<MessageModifier, const MessageModifierContext&>::CreateOrDie(
      modifier_name_or_alias, context);
}

absl::Duration MessageModifier::GetOffset(
    const google::protobuf::Message& lite_msg) {
  const auto& header = LiteMsgConverter::Get().GetLiteMessageHeader(lite_msg);
  return absl::FromUnixMicros(header.timestamp()) -
         context_.scenario_start_time;
}

}  // namespace qcraft
