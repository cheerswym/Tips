#ifndef ONBOARD_LITE_TRANSPORT_MESSAGE_SHM_MESSAGE_HUB_H_
#define ONBOARD_LITE_TRANSPORT_MESSAGE_SHM_MESSAGE_HUB_H_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport/message/message_hub.h"
#include "onboard/lite/transport/shm/condition_notifier.h"
#include "onboard/lite/transport/shm/shm_transmitter.h"

namespace qcraft {
class ShmMessageHub : public MessageHub {
 public:
  explicit ShmMessageHub(LiteModuleName module_name);

  ~ShmMessageHub() {
    is_running_.store(false);
    notifier_->Shutdown();
    thread_.join();
  }

  bool DispatchMessage(
      const std::string& domain_channel,
      std::shared_ptr<google::protobuf::Message> message) override;

  void RegisterDomainChannel(const std::string& domain_channel,
                             MessageHub::PushMsgCallback func) override;

  // Reset message hub to initial state.
  void Reset() override;

  void StartListening();

 private:
  void ListenThread();

  void ReadMessage(const shm::ReadableInfo& readable_info, const int& index);

  absl::Mutex segements_mu_;

  std::thread thread_;
  std::unique_ptr<shm::ShmTransmitter> transmitter_;
  std::unique_ptr<shm::ConditionNotifier> notifier_;

  const LiteModuleName module_name_;
  std::atomic<bool> is_running_;
  absl::Mutex mu_;
  std::unordered_map<
      size_t, std::pair<int64_t, std::vector<MessageHub::PushMsgCallback>>>
      push_msg_callbacks_ GUARDED_BY(mu_);
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_MESSAGE_SHM_MESSAGE_HUB_H_
