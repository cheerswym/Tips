#ifndef ONBOARD_LITE_TRANSPORT_MESSAGE_MESSAGE_HUB_H_
#define ONBOARD_LITE_TRANSPORT_MESSAGE_MESSAGE_HUB_H_

#include <functional>
#include <memory>
#include <string>
#include <variant>

#include "google/protobuf/message.h"

class MessageHub {
 public:
  typedef std::function<void(
      const std::string&,
      const std::variant<std::shared_ptr<google::protobuf::Message>,
                         std::shared_ptr<std::string>>& message)>

      PushMsgCallback;

  virtual ~MessageHub() {}

  // Dispatch messages.
  virtual bool DispatchMessage(
      const std::string& domain_channel,
      std::shared_ptr<google::protobuf::Message> message) = 0;
  // Register donmain channel.
  virtual void RegisterDomainChannel(const std::string& domain_channel,
                                     PushMsgCallback func) = 0;
  // Reset to clean up all registered callbacks.
  virtual void Reset() = 0;
};

#endif  // ONBOARD_LITE_TRANSPORT_MESSAGE_MESSAGE_HUB_H_
