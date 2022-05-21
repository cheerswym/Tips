#ifndef ONBOARD_LITE_TRANSPORT_SHM_SHM_TRANSMITTER_H_
#define ONBOARD_LITE_TRANSPORT_SHM_SHM_TRANSMITTER_H_

#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include "google/protobuf/message.h"
#include "onboard/lite/transport/shm/condition_notifier.h"

namespace qcraft {
namespace shm {
class ShmTransmitter {
 public:
  ShmTransmitter(uint64_t module_id);
  ~ShmTransmitter();

  void Enable();
  void Disable();

  bool Transmit(const std::string& domain_channel,
                std::shared_ptr<google::protobuf::Message> message,
                int subscriber_num);

 private:
  std::unique_ptr<ConditionNotifier> notifier_;
  bool enabled_;
  [[maybe_unused]] uint64_t module_id_;
};
}  // namespace shm
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHM_SHM_TRANSMITTER_H_
