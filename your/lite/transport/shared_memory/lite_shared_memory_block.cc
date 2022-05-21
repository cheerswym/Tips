#include "onboard/lite/transport/shared_memory/lite_shared_memory_block.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "glog/logging.h"
#include "google/protobuf/message.h"
#include "onboard/base/base.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
SharedMemoryBlock::~SharedMemoryBlock() {
  shm_msg_manager_->ReleaseShmBlock(shm_msg_metadata_);
}

// Lite share message.
SharedMemoryBlock::SharedMemoryBlock(SharedMemoryManager* shm_msg_manager,
                                     void* buffer, int32_t buffer_size,
                                     const ShmMessageMetadata& shm_message)
    : shm_msg_manager_(shm_msg_manager),
      buffer_(buffer),
      buffer_size_(buffer_size),
      shm_msg_metadata_(shm_message),
      reader_or_writer_(shm_message.reader_or_writer()) {}

void* SharedMemoryBlock::MutableBuffer() {
  if (reader_or_writer_) {
    LOG(ERROR) << "Trying to get a mutable buffer from a read only memory";
    return NULL;
  }
  return buffer_;
}

const void* SharedMemoryBlock::Buffer() const { return buffer_; }

int32 SharedMemoryBlock::BufferSize() const { return buffer_size_; }

const ShmMessageMetadata& SharedMemoryBlock::ShmMsgMetadata() const {
  return shm_msg_metadata_;
}

ShmMessageMetadata* SharedMemoryBlock::MutableShmMsgMetadata() {
  return &shm_msg_metadata_;
}

}  // namespace qcraft
