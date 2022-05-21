#include "onboard/lite/transport/message/shm_message_hub.h"

#include <memory>
#include <utility>
#include <vector>

#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/counter.h"
#include "onboard/lite/check.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"
#include "onboard/lite/transport/shm/shm_debug_util.h"
#include "onboard/utils/thread_util.h"

namespace qcraft {
namespace {
const int32 kShmTagNumber = 6;
}
// Share memory based hub implementation
ShmMessageHub::ShmMessageHub(LiteModuleName module_name)
    : module_name_(module_name), is_running_(false) {
  transmitter_ =
      std::make_unique<shm::ShmTransmitter>(static_cast<int32>(module_name_));
  notifier_ = std::make_unique<shm::ConditionNotifier>();
}

bool ShmMessageHub::DispatchMessage(
    const std::string& domain_channel,
    std::shared_ptr<google::protobuf::Message> message) {
  int reference_count_need_consume =
      shm::ShmManager::Instance()->GetSubscriberNum(domain_channel);
  bool success = transmitter_->Transmit(domain_channel, message,
                                        reference_count_need_consume);
  CHECK(success) << "tranmitter failed " << message->DebugString();
  return success;
}

void ShmMessageHub::StartListening() {
  notifier_->ReaderCatchup();
  thread_ = std::thread(&ShmMessageHub::ListenThread, this);
}

void ShmMessageHub::ListenThread() {
  is_running_.store(true);
  shm::ReadableInfo readable_info;
  int index;
  QSetThreadName("ShmMsgListening");
  while (is_running_.load()) {
    if (!notifier_->Read(10, &readable_info, &index)) {
      continue;
    }
    if (is_running_.load()) {
      ReadMessage(readable_info, index);
    }
  }
}

void ShmMessageHub::ReadMessage(const shm::ReadableInfo& readable_info,
                                const int& index) {
  QCOUNTER_SPAN("ShmMessageHub::ReadMessage");
  absl::ReaderMutexLock l(&mu_);
  const size_t domain_channel_hash = readable_info.domain_channel_hash;
  auto iter = push_msg_callbacks_.find(domain_channel_hash);
  // I am not interested about this channel, skip.
  if (iter == push_msg_callbacks_.end()) {
    return;
  }

  VLOG(5) << "ReadMessage:" << module_name_ << ":"
          << shm::DebugUtil::Instance()->TryFindChannelNameWithHash(
                 domain_channel_hash);
  if (shm::ShmManager::Instance()->GetRefCount(readable_info.handle) <= 0) {
    QLOG(ERROR)
        << "[LITE] !!! Transit error: GetRefCount is zero, module_name: "
        << LiteModuleName_Name(module_name_)
        << " handle:" << readable_info.handle << " handle ref:"
        << shm::ShmManager::Instance()->GetRefCount(readable_info.handle)
        << " hash:"
        << shm::DebugUtil::Instance()->TryFindChannelNameWithHash(
               domain_channel_hash)
        << ", index:" << index << ", index_ref_conut:"
        << shm::ShmManager::Instance()->GetIndexRefCount(index);
    return;
  }

  void* addr =
      shm::ShmManager::Instance()->GetAddressFromHandle(readable_info.handle);
  shm::LiteShmHeader lite_shm_header;

  CHECK(lite_shm_header.DeserializeFrom(reinterpret_cast<char*>(addr)))
      << "Failed to Deserialize header";
  if (!IsDSimMode()) {
    if (lite_shm_header.timestamp() <= iter->second.first) {
      QLOG(ERROR)
          << "RegisterDomainChannel time is not earlier than the publish time:"
          << lite_shm_header.timestamp() << " <= " << iter->second.first;
      return;
    }
  }

  const std::vector<PushMsgCallback>* callbacks = &(iter->second.second);
  std::shared_ptr<std::string> message = std::make_shared<std::string>();
  message->resize(readable_info.msg_size);
  memcpy(message->data(),
         reinterpret_cast<char*>(addr) + readable_info.header_size,
         readable_info.msg_size);
  if (lite_shm_header.tag_number() != kShmTagNumber) {
    shm::ShmManager::Instance()->DecreaseIndexRefCount(index,
                                                       callbacks->size());
    shm::ShmManager::Instance()->DecreaseRefCount(readable_info.handle,
                                                  callbacks->size());
  }

  for (const auto& cb : *callbacks) {
    VLOG(5) << "callback to callbacks:" << module_name_ << " :"
            << shm::DebugUtil::Instance()->TryFindChannelNameWithHash(
                   domain_channel_hash);
    cb(lite_shm_header.domain_channel(), message);
  }
}

void ShmMessageHub::RegisterDomainChannel(const std::string& domain_channel,
                                          PushMsgCallback func) {
  VLOG(5) << "ShmMessageHub::RegisterDomainChannel:" << domain_channel;
  absl::MutexLock l(&mu_);
  const auto timestamp = shm::ShmManager::Instance()->Subscribe(domain_channel);
  const size_t domain_channel_hash = StringHash(domain_channel);
  VLOG(5) << "RegisterDomainChannel with callback: {" << domain_channel_hash
          << ",\"" << domain_channel << "\"}";
  auto& pair = push_msg_callbacks_[domain_channel_hash];
  pair.first = timestamp;
  pair.second.emplace_back(std::move(func));
  VLOG(5) << "callback size:" << pair.second.size();
  VLOG(5) << "ShmMessageHub::RegisterDomainChannel end";
}

void ShmMessageHub::Reset() {
  absl::MutexLock l(&mu_);
  push_msg_callbacks_.clear();
}
}  // namespace qcraft
