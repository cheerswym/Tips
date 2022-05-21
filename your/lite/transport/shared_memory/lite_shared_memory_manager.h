#ifndef ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_MANAGER_H_
#define ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_MANAGER_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "google/protobuf/message.h"
#include "onboard/base/base.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_block.h"
#include "onboard/utils/map_util.h"
namespace qcraft {
class SharedMemoryManager {
 public:
  struct Options {
    int64_t buffer_size;
    // Used for tracking channel memory allocate status.
    std::string channel;
    ShmMsgType shm_msg_type;
  };

  virtual ~SharedMemoryManager() {}

  // Allocate memory to write, if failed returns nullptr.
  virtual std::unique_ptr<SharedMemoryBlock> AllocateMemToWrite(
      const SharedMemoryManager::Options& options) = 0;

  // Before Send shared memory msg to the channel, update related counter to
  // make sure the msg is not destroy during lite transportation.
  virtual void BeforeSendSharedMemoryMsg(
      const ShmMessageMetadata& shm_message_proto) = 0;

  // After Received shared memory msg from the channel, update related counter
  // to make sure to release the hold of the msg, so once the ref count reduce
  // to 0, the memory will be properly released.
  virtual void AfterReceiveSharedMemoryMsg(
      const ShmMessageMetadata& shm_message_proto) = 0;

  virtual void RegisterShmMsgSubscriber(const std::string& channel,
                                        const std::string& domain) = 0;

  virtual std::unique_ptr<SharedMemoryBlock> CreateMemToRead(
      const ShmMessageMetadata& shm_message_proto) = 0;

  // Release shm block back to share memory pool.
  virtual void ReleaseShmBlock(const ShmMessageMetadata& shm_message_proto) = 0;

  // Returns number of avaiable share memory blocks
  virtual int32 NumOfAvaibleShmBlocks() = 0;

  // Reset internal state.
  virtual void Reset() = 0;

  // // This function is not thread-safe, should only call it before starting
  // // modules.
  virtual void TEST_SetRegisteredShmMsgReaderCount(
      const std::map<std::pair<std::string, std::string>, int32>&
          registered_reader_channel_domain_cnt) = 0;

  virtual int32 GetNumberOfRegisteredReaders(const std::string& channel,
                                             const std::string& domain) = 0;

  // Return how much memory is used.
  virtual int64 GetUsedMemory() = 0;
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_MANAGER_H_
