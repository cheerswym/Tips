#ifndef ONBOARD_LITE_TRANSPORT_MESSAGE_INNER_MESSAGE_HUB_H_
#define ONBOARD_LITE_TRANSPORT_MESSAGE_INNER_MESSAGE_HUB_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "onboard/lite/transport/message/message_hub.h"

namespace qcraft {

class InnerMessageHub : public MessageHub {
 public:
  bool DispatchMessage(
      const std::string& domain_channel,
      std::shared_ptr<google::protobuf::Message> message) override;

  void RegisterDomainChannel(const std::string& domain_channel,
                             MessageHub::PushMsgCallback func) override;

  // Reset message hub to initial state.
  void Reset() override;

 private:
  absl::Mutex mu_;

  std::unordered_map<std::string, std::vector<MessageHub::PushMsgCallback>>
      push_msg_callbacks_ GUARDED_BY(mu_);
};

InnerMessageHub* GlobalInnerMessageHub();

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_MESSAGE_INNER_MESSAGE_HUB_H_
