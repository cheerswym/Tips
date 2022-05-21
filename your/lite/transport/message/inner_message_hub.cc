#include "onboard/lite/transport/message/inner_message_hub.h"

#include "glog/logging.h"

namespace qcraft {

bool InnerMessageHub::DispatchMessage(
    const std::string& domain_channel,
    std::shared_ptr<google::protobuf::Message> message) {
  absl::ReaderMutexLock l(&mu_);
  auto iter = push_msg_callbacks_.find(domain_channel);
  if (iter == push_msg_callbacks_.end()) {
    LOG_EVERY_N(WARNING, 10000) << "No subscribers for " << domain_channel;
    return true;
  }
  const std::vector<PushMsgCallback>& push_msg_callbacks = iter->second;
  for (const auto& cb : push_msg_callbacks) {
    cb(domain_channel, message);
  }
  return true;
}

void InnerMessageHub::RegisterDomainChannel(const std::string& domain_channel,
                                            PushMsgCallback func) {
  absl::MutexLock l(&mu_);
  std::vector<PushMsgCallback>& callbacks = push_msg_callbacks_[domain_channel];
  callbacks.emplace_back(std::move(func));
}

void InnerMessageHub::Reset() {
  absl::MutexLock l(&mu_);
  push_msg_callbacks_.clear();
}

InnerMessageHub* GlobalInnerMessageHub() {
  static InnerMessageHub* global_message_hub = new InnerMessageHub();
  return global_message_hub;
}
}  // namespace qcraft
