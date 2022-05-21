#include "onboard/lite/modifier/create_sim_messages_module.h"

#include <memory>

#include "absl/strings/str_cat.h"
#include "offboard/simulation/proto/scenario.pb.h"
#include "onboard/lite/lite_module.h"
#include "onboard/lite/module_config_helper.h"
#include "onboard/proto/lite_msg.pb.h"
namespace qcraft {

CreateSimMessagesModule::CreateSimMessagesModule(LiteClientBase* lite_client)
    : LiteModule(lite_client) {}

void CreateSimMessagesModule::OnInit() {}

void CreateSimMessagesModule::OnSubscribeChannels() {
  DisableInputOutputChecker();
  const auto& channel_domain_to_field_name =
      GetOutputsFromModuleConfigs(LoadModuleConfigsFromAllDeclared());
  QCHECK(!channel_domain_to_field_name.empty()) << "No module configs provided";
  for (const auto& [channel_domain, field_name] :
       channel_domain_to_field_name) {
    if (IsInLiteMsgBlackList(field_name)) {
      VLOG(2) << "Field " << field_name
              << " is in blacklist. Will not subscribe.";
      continue;
    }

    if (field_name == "shm_message_metadata") continue;

    VLOG(1) << "Create sim messages module subscribes " << channel_domain.first
            << ":" << channel_domain.second << ":" << field_name;
    SubscribeMsgWithCallback(
        channel_domain.first, channel_domain.second, field_name,
        [this](std::shared_ptr<const LiteMsgWrapper> lite_msg) {
          this->LiteMsgReceiver(lite_msg);
        },
        nullptr);
  }
}

void CreateSimMessagesModule::OnSetUpTimers() {}

void CreateSimMessagesModule::LiteMsgReceiver(
    std::shared_ptr<const LiteMsgWrapper> lite_msg) {
  const auto channel_domain =
      LiteMsgConverter::Get().GetLiteMsgChannelDomain(*lite_msg);
  if (IsOnboardMode()) {
    return;
  }

  LiteMsgWrapper lite_msg_copy = *lite_msg;
  auto* header =
      LiteMsgConverter::Get().GetMutableLiteMsgWrapperHeader(&lite_msg_copy);
  if (!header->HasExtension(SimLiteHeaderProto::sim_lite_header)) return;

  auto* sim_header =
      header->MutableExtension(SimLiteHeaderProto::sim_lite_header);
  for (auto& request : *sim_header->mutable_create_message_requests()) {
    QLOG_IF_NOT_OK(WARNING,
                   PublishLiteMsgWithMeta(request.channel(), request.domain(),
                                          request.mutable_msg()));
  }
}

}  // namespace qcraft
