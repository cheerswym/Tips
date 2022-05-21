#ifndef ONBOARD_LITE_LITE_SHM_MESSAGE_H_
#define ONBOARD_LITE_LITE_SHM_MESSAGE_H_

// TODO(kun):
// 1, Support multi process
// 2, Share memory block Garbage collection
// 3, Gen status of memory status
// 4, Set memory pool reuse and max limit.
// 5, Solve the multiple write with same memory and expected reader issue.
// 6, Monitor count of reserved shm block, in case user reserve too many shm
// blocks unexpectly.

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "onboard/base/base.h"
#include "onboard/global/trace.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager_factory.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

// Not thread safe.
class ShmMessage {
 public:
  virtual ~ShmMessage() { shm_block_.reset(); }

  static std::unique_ptr<ShmMessage> CreateToWrite(
      int32 buffer_size, const ShmMsgType shm_msg_type,
      const std::string& channel, const std::string& domain = "") {
    SharedMemoryManager::Options options;
    options.buffer_size = buffer_size;
    return CreateToWrite(options, shm_msg_type, channel, domain);
  }

  // Create a share memory message with specific channel and domain.
  template <typename T>
  static std::unique_ptr<ShmMessage> CreateToWrite(
      const ShmMsgType shm_msg_type, const std::string& channel,
      const std::string& domain = "") {
    SharedMemoryManager::Options options;
    options.buffer_size = T::ExpectedSize();
    return CreateToWrite(options, shm_msg_type, channel, domain);
  }

  static std::shared_ptr<ShmMessage> CreateToRead(
      const ShmMessageMetadata& shm_message_proto) {
    std::unique_ptr<SharedMemoryBlock> shm_block =
        ShmFactory::GetShmManager()->CreateMemToRead(shm_message_proto);
    if (shm_block == nullptr) {
      LOG(ERROR) << "shm_block is nullptr, create to read failed with "
                 << shm_message_proto.DebugString();
      return nullptr;
    }
    // Cannot use std::make_shared here as this is a private c'tor.
    return std::shared_ptr<ShmMessage>(new ShmMessage(std::move(shm_block)));
  }

  template <typename T>
  T* mutable_value() {
    return T::CastToPointer(shm_block_->MutableBuffer());
  }

  template <typename T>
  const T& value() const {
    return T::CastToReference(shm_block_->Buffer());
  }

  void* mutable_buffer() { return shm_block_->MutableBuffer(); }

  const void* buffer() const { return shm_block_->Buffer(); }

  int32 buffer_size() const { return shm_block_->BufferSize(); }

  int handle() const {
    CHECK(shm_block_->ShmMsgMetadata().has_cross_proc());
    return shm_block_->ShmMsgMetadata().cross_proc().handle();
  }

  const ShmMessageMetadata& shm_msg_metadata() const {
    return shm_block_->ShmMsgMetadata();
  }

  ShmMessageMetadata* mutable_shm_msg_metadata() {
    return shm_block_->MutableShmMsgMetadata();
  }

  void CopyShmDataToBuffer(void* data) {
    memcpy(data, shm_block_->Buffer(), shm_block_->BufferSize());
  }

  void CopyShmData(std::string* data) {
    data->resize(shm_block_->BufferSize());
    memcpy(data->data(), shm_block_->Buffer(), shm_block_->BufferSize());
  }

 private:
  std::unique_ptr<SharedMemoryBlock> shm_block_;

  static std::unique_ptr<ShmMessage> CreateToWrite(
      const SharedMemoryManager::Options& options,
      const ShmMsgType shm_msg_type, const std::string& channel,
      const std::string& domain = "") {
    SharedMemoryManager::Options copy_options = options;
    copy_options.channel = channel;
    copy_options.shm_msg_type = shm_msg_type;
    std::unique_ptr<ShmMessage> value;
    std::unique_ptr<SharedMemoryBlock> shm_block =
        ShmFactory::GetShmManager()->AllocateMemToWrite(copy_options);
    if (shm_block == nullptr) {
      return nullptr;
    }
    value.reset(new ShmMessage(std::move(shm_block)));
    value->mutable_shm_msg_metadata()->mutable_header()->set_channel(channel);
    value->mutable_shm_msg_metadata()->mutable_header()->set_domain(domain);
    value->mutable_shm_msg_metadata()->set_shm_msg_type(shm_msg_type);
    return value;
  }

  explicit ShmMessage(std::unique_ptr<SharedMemoryBlock> shm_block)
      : shm_block_(std::move(shm_block)) {}

  ShmMessage(const ShmMessage&) = delete;
  ShmMessage(const ShmMessage&&) = delete;
  ShmMessage& operator=(const ShmMessage&) = delete;
  ShmMessage& operator=(const ShmMessage&&) = delete;
};

// If use define a data type XXX, should put REGISTER_SHM_MSG(XXX) inside the
// struct or class. Otherwise provide three staic functions for the type.
//
// static int32 ExpectedSize()
// static Type* CastToPointer(void* buffer)
// static const Type& CastToReference(const void* buffer)
//
#define REGISTER_SHM_MSG_WITH_SIZE(Type, Size)             \
  static int32 ExpectedSize() { return (Size); }           \
  static Type* CastToPointer(void* buffer) {               \
    return static_cast<Type*>(buffer);                     \
  }                                                        \
  static const Type& CastToReference(const void* buffer) { \
    return *static_cast<const Type*>(buffer);              \
  }

// Register share memory type with default size, use sizeof(Type).
// It requires the Type using a continuous memory space.
#define REGISTER_SHM_MSG(Type) REGISTER_SHM_MSG_WITH_SIZE(Type, sizeof(Type))

// Register share memory type without fix size, use will provide buffer size
// when call CreatToWrite.
#define REGISTER_SHM_MSG_WITHOUT_SIZE(Type)                \
  static Type* CastToPointer(void* buffer) {               \
    return static_cast<Type*>(buffer);                     \
  }                                                        \
  static const Type& CastToReference(const void* buffer) { \
    return *static_cast<const Type*>(buffer);              \
  }

}  // namespace qcraft

#endif  // ONBOARD_LITE_LITE_SHM_MESSAGE_H_
