#ifndef ONBOARD_LITE_LITE_TRANSPORT_H_
#define ONBOARD_LITE_LITE_TRANSPORT_H_

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "onboard/lite/lite_callbacks.h"
#include "onboard/lite/modifier/message_modifier_man.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport.h"
#include "onboard/lite/transport/message/message_hub.h"

namespace qcraft {
class LiteTransport : public Transport {
 public:
  explicit LiteTransport(MessageHub* message_hub);

  // Receives the next message in the message queue and invokes one of its
  // associated subscriber callback. If the queue is empty, block for at most
  // `duration`. Returns true if a message has been received. If `delayed_subs`
  // is non-empty, it will be populated with all delayed subscribers.
  bool ReceiveNextMessage(absl::Duration duration) override;

  bool PublishMessage(
      const std::string& channel, const std::string& domain,
      std::unique_ptr<google::protobuf::Message> message) override;

  void SubscribeMessage(const std::string& channel, const std::string& domain,
                        std::unique_ptr<SubCallback> sub_callback) override;

  void SetMsgModifierMan(MessageModifierMan* msg_modifier_man) override;

  void Stop() override;

  MessageHub* GetMessageHub() const;

 private:
  struct ReceiveMsgCallback {
    std::variant<std::shared_ptr<google::protobuf::Message>,
                 std::shared_ptr<std::string>>
        message;
    SubCallback* callback;
    std::string domain_channel;
    absl::Time recv_time_;
  };

  void PushMessageCallback(
      SubCallback* callback, const std::string& domain_channel,
      const std::variant<std::shared_ptr<google::protobuf::Message>,
                         std::shared_ptr<std::string>>& message);

  static bool IsNotEmpty(std::queue<ReceiveMsgCallback>* msg_queue) {
    return !msg_queue->empty();
  }

 private:
  // Not owned.
  MessageModifierMan* msg_modifier_man_ = nullptr;

  MessageHub* const message_hub_;

  absl::Mutex reg_callback_mu_;
  std::vector<std::unique_ptr<SubCallback>> registered_callbacks_
      GUARDED_BY(reg_callback_mu_);

  absl::Mutex msg_mu_;
  std::queue<ReceiveMsgCallback> message_callback_ GUARDED_BY(msg_mu_);
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_LITE_TRANSPORT_H_
