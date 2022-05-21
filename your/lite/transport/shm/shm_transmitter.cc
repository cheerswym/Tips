#include "onboard/lite/transport/shm/shm_transmitter.h"

#include "onboard/global/counter.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/transport/shm/condition_notifier.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
namespace {
const int32 kShmTagNumber = 6;
const uint64 kPlaceholder = std::numeric_limits<long>::max();
}  // namespace
namespace shm {
ShmTransmitter::ShmTransmitter(uint64_t module_id)
    : enabled_(false), module_id_(module_id) {
  Enable();
}

ShmTransmitter::~ShmTransmitter() { Disable(); }

void ShmTransmitter::Enable() {
  if (this->enabled_) {
    return;
  }
  this->enabled_ = true;
  notifier_ = std::make_unique<ConditionNotifier>();
}

void ShmTransmitter::Disable() {
  if (this->enabled_) {
    this->enabled_ = false;
  }
  notifier_->Shutdown();
}

bool ShmTransmitter::Transmit(
    const std::string& domain_channel,
    std::shared_ptr<google::protobuf::Message> message, int subscriber_num) {
  if (!this->enabled_) {
    LOG(FATAL) << "not enable.";
    return false;
  }

  if (subscriber_num == 0) {
    return true;
  }
  LiteShmHeader lite_shm_header(
      domain_channel, LiteMsgConverter::Get().GetLiteMsgTagNumber(*message),
      LiteMsgConverter::Get().GetLiteMessageHeader(*message));

  size_t domain_channel_hash = StringHash(domain_channel);
  void* buf = nullptr;
  std::size_t msg_size = 0;
  std::size_t header_size = lite_shm_header.ByteSize();
  bip::managed_shared_memory::handle_t handle = 0;

  // get index
  uint64_t current_write_seq, current_write_index;
  if (!notifier_->GetWriteIndex(&current_write_seq, &current_write_index)) {
    return false;
  }
  if (lite_shm_header.tag_number() == kShmTagNumber) {
    ShmMessageMetadata* shm_msg_meta =
        dynamic_cast<ShmMessageMetadata*>(message.get());
    auto cross_proc = shm_msg_meta->mutable_cross_proc();
    cross_proc->set_msg_handle(kPlaceholder);
    cross_proc->set_index(current_write_index);
    cross_proc->set_msg_domain_channel_hash(domain_channel_hash);
    buf =
        ShmManager::Instance()->Allocate(header_size + message->ByteSizeLong());
    CHECK(buf != nullptr)
        << "Failed to allocate share memory in BoostShmAllocator";

    handle = ShmManager::Instance()->GetHandleFromAddress(buf);
    cross_proc->set_msg_handle(handle);
    msg_size = message->ByteSizeLong();
  } else {
    msg_size = message->ByteSizeLong();

    buf = ShmManager::Instance()->Allocate(header_size + msg_size);
    CHECK(buf != nullptr)
        << "Failed to allocate share memory in BoostShmAllocator";

    handle = ShmManager::Instance()->GetHandleFromAddress(buf);
  }
  // write header
  lite_shm_header.SerializeTo(reinterpret_cast<char*>(buf));

  // write msg
  char* msg_addr = reinterpret_cast<char*>(buf) + header_size;
  message->SerializeToArray(msg_addr, static_cast<int>(msg_size));

  ReadableInfo readable_info{handle, header_size, msg_size,
                             domain_channel_hash};
  ShmManager::Instance()->IncreaseRefCount(handle, subscriber_num);

  bool res = notifier_->Write(readable_info, subscriber_num, current_write_seq,
                              current_write_index);
  return res;
}

}  // namespace shm
}  // namespace qcraft
