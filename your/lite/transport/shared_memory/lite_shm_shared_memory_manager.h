#ifndef ONBOARD_LITE_TRANSPORT_SHM_LITE_SHM_SHARED_MEMORY_MANAGER_H_
#define ONBOARD_LITE_TRANSPORT_SHM_LITE_SHM_SHARED_MEMORY_MANAGER_H_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_block.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager.h"

namespace qcraft {

class ShmSharedMemoryManager : public SharedMemoryManager {
 public:
  std::unique_ptr<SharedMemoryBlock> AllocateMemToWrite(
      const SharedMemoryManager::Options& options) override;

  void BeforeSendSharedMemoryMsg(
      const ShmMessageMetadata& shm_message_proto) override;

  void AfterReceiveSharedMemoryMsg(
      const ShmMessageMetadata& shm_message_proto) override;

  std::unique_ptr<SharedMemoryBlock> CreateMemToRead(
      const ShmMessageMetadata& shm_msg_metadata) override;

  void RegisterShmMsgSubscriber(const std::string& channel,
                                const std::string& domain) override;

  void ReleaseShmBlock(const ShmMessageMetadata& shm_msg_metadata) override;

  int32 NumOfAvaibleShmBlocks() override;

  void Reset() override;

  int32 GetNumberOfRegisteredReaders(const std::string& channel,
                                     const std::string& domain) override;

  void TEST_SetRegisteredShmMsgReaderCount(
      const std::map<std::pair<std::string, std::string>, int32>&
          registered_reader_channel_domain_cnt) override;

  int64 GetUsedMemory() override;

 private:
  absl::Mutex mu_;
};

}  // namespace qcraft

#endif  // ONBOARD_LITE_TRANSPORT_SHM_LITE_SHM_SHARED_MEMORY_MANAGER_H_
