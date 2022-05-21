#ifndef ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_INNER_PROCESS_SHARED_MEMORY_MANAGER_H_  // NOLINT
#define ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_INNER_PROCESS_SHARED_MEMORY_MANAGER_H_  // NOLINT

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_block.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager.h"

namespace qcraft {
class InnerProcessSharedMemoryManager : public SharedMemoryManager {
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

  int64 GetUsedMemory() override;

  void Reset() override {
    absl::WriterMutexLock l(&shm_reader_mu_);
    shm_reader_channel_domain_cnt_.clear();
  }

  int32 GetNumberOfRegisteredReaders(const std::string& channel,
                                     const std::string& domain) override;
  void TEST_SetRegisteredShmMsgReaderCount(
      const std::map<std::pair<std::string, std::string>, int32>&
          registered_reader_channel_domain_cnt) override {
    absl::WriterMutexLock l(&shm_reader_mu_);
    shm_reader_channel_domain_cnt_ = registered_reader_channel_domain_cnt;
  }

 private:
  struct RefInfo {
    int32 ref_count;
    int32 created_readers;
    int32 min_created_readers;
    SharedMemoryManager::Options options;
  };

  std::string DumpAllocatedRamInfo() EXCLUSIVE_LOCKS_REQUIRED(mu_);

  const int64 kMaxRamPool = 8LL << 30;  // 8G
  mutable absl::Mutex shm_reader_mu_;
  std::map<std::pair<std::string, std::string>, int32>
      shm_reader_channel_domain_cnt_ GUARDED_BY(shm_reader_mu_);
  absl::Mutex mu_;
  std::unordered_map<uint64, RefInfo> key_to_ref_info_ GUARDED_BY(mu_);
  std::unordered_map<std::string, int64> channel_to_allocated_ram_
      GUARDED_BY(mu_);
  int64 total_allocated_ram_ GUARDED_BY(mu_) = 0;
};
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_INNER_PROCESS_SHARED_MEMORY_MANAGER_H_
