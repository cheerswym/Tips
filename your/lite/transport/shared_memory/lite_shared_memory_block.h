#ifndef ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_BLOCK_H_
#define ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_BLOCK_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "onboard/base/base.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

class SharedMemoryManager;

// Not thread safe.
class SharedMemoryBlock {
 public:
  ~SharedMemoryBlock();

  SharedMemoryBlock(SharedMemoryManager* shm_msg_manager, void* buffer,
                    int32_t buffer_size, const ShmMessageMetadata& shm_message);

  void* MutableBuffer();

  const void* Buffer() const;

  int32 BufferSize() const;

  const ShmMessageMetadata& ShmMsgMetadata() const;

  ShmMessageMetadata* MutableShmMsgMetadata();

 private:
  SharedMemoryManager* const shm_msg_manager_;
  // Won't never own this pointer.
  void* const buffer_;
  const int32 buffer_size_;
  ShmMessageMetadata shm_msg_metadata_;
  const bool reader_or_writer_;

  // No copy constructor and copy assignment?
  SharedMemoryBlock(const SharedMemoryBlock&) = delete;
  SharedMemoryBlock(const SharedMemoryBlock&&) = delete;
  SharedMemoryBlock& operator=(const SharedMemoryBlock&) = delete;
  SharedMemoryBlock& operator=(const SharedMemoryBlock&&) = delete;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_BLOCK_H_
