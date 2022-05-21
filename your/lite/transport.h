#ifndef ONBOARD_LITE_TRANSPORT_H_
#define ONBOARD_LITE_TRANSPORT_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "onboard/lite/lite_callbacks.h"
#include "onboard/lite/modifier/message_modifier_man.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport/message/message_hub.h"
#include "onboard/proto/execution_issue.pb.h"

namespace qcraft {
class Transport {
 public:
  virtual ~Transport() {}

  // Return false, if no message is in pending queue after duration expired or
  // duration < ZeroDuration().
  virtual bool ReceiveNextMessage(absl::Duration duration) = 0;

  virtual bool PublishMessage(
      const std::string& channel, const std::string& domain,
      std::unique_ptr<google::protobuf::Message> message) = 0;

  virtual void SubscribeMessage(const std::string& channel,
                                const std::string& domain,
                                std::unique_ptr<SubCallback> sub_callback) = 0;

  virtual void SetMsgModifierMan(MessageModifierMan* msg_modifier_man) = 0;

  virtual void Stop() = 0;
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_H_
