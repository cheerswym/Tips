#ifndef ONBOARD_LITE_MODIFIER_CREATE_SIM_MESSAGES_MODULE_H_
#define ONBOARD_LITE_MODIFIER_CREATE_SIM_MESSAGES_MODULE_H_

#include <memory>

#include "onboard/lite/lite_module.h"
#include "onboard/proto/lite_msg.pb.h"

namespace qcraft {

// A module that publishes on-demand lite messages in simulation.
class CreateSimMessagesModule : public LiteModule {
 public:
  explicit CreateSimMessagesModule(LiteClientBase* lite_client);
  void OnInit() override;
  void OnSubscribeChannels() override;
  void OnSetUpTimers() override;

 private:
  void LiteMsgReceiver(std::shared_ptr<const LiteMsgWrapper> lite_msg);
};
REGISTER_LITE_MODULE(CreateSimMessagesModule);

}  // namespace qcraft

#endif  // ONBOARD_LITE_MODIFIER_CREATE_SIM_MESSAGES_MODULE_H_
