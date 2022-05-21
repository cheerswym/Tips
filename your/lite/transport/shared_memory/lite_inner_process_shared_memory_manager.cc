#include "onboard/lite/transport/shared_memory/lite_inner_process_shared_memory_manager.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "google/protobuf/message.h"
#include "onboard/base/base.h"
#include "onboard/global/counter.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

std::unique_ptr<SharedMemoryBlock>
InnerProcessSharedMemoryManager::AllocateMemToWrite(
    const SharedMemoryManager::Options& options) {
  absl::MutexLock l(&mu_);
  if (total_allocated_ram_ + options.buffer_size >= kMaxRamPool) {
    LOG(FATAL)
        << "No available ram in share memory pool. Please check if any shared "
           "memory messages are not deconstructed correctly. Now It's over "
        << (kMaxRamPool >> 20) << "M.\n Allocated ram info: \n"
        << DumpAllocatedRamInfo();
  }
  total_allocated_ram_ += options.buffer_size;
  channel_to_allocated_ram_[options.channel] += options.buffer_size;
  QCOUNTER("allocated_shm_mb", total_allocated_ram_ >> 20);

  void* buffer = new char[options.buffer_size];
  if (buffer == nullptr) {
    return nullptr;
  }
  uint64 void_addr = reinterpret_cast<uint64>(buffer);
  ShmMessageMetadata shm_msg_metadata;
  shm_msg_metadata.set_buffer_size(options.buffer_size);
  shm_msg_metadata.set_reader_or_writer(false);
  auto* metadata = shm_msg_metadata.mutable_inner_proc();
  metadata->set_void_addr(void_addr);
  QCHECK(!ContainsKey(key_to_ref_info_, void_addr));
  RefInfo* ref_info = &key_to_ref_info_[void_addr];
  ref_info->ref_count = 1;
  ref_info->created_readers = 0;
  ref_info->min_created_readers = 0;
  ref_info->options = options;

  // Reset created readers to zero.
  return std::make_unique<SharedMemoryBlock>(this, buffer, options.buffer_size,
                                             shm_msg_metadata);
}

std::string InnerProcessSharedMemoryManager::DumpAllocatedRamInfo() {
  std::string debug_info;
  for (const auto& iter : channel_to_allocated_ram_) {
    debug_info +=
        absl::StrCat(iter.first, " allocated ", iter.second, " bytes\n");
  }
  return debug_info;
}

void InnerProcessSharedMemoryManager::BeforeSendSharedMemoryMsg(
    const ShmMessageMetadata& shm_msg_metadata) {
  const auto& metadata = shm_msg_metadata.inner_proc();
  QCHECK(metadata.void_addr());
  uint64 void_addr = metadata.void_addr();
  absl::MutexLock l(&mu_);
  auto sub_num = GetNumberOfRegisteredReaders(
      shm_msg_metadata.header().channel(), shm_msg_metadata.header().domain());
  RefInfo* ref_info = &FindOrDie(key_to_ref_info_, void_addr);
  ref_info->min_created_readers += sub_num;
  VLOG(4) << "BeforeSendSharedMemoryMsg: required:"
          << ref_info->min_created_readers
          << " created: " << ref_info->created_readers;
}

void InnerProcessSharedMemoryManager::AfterReceiveSharedMemoryMsg(
    const ShmMessageMetadata& shm_msg_metadata) {
  const auto& metadata = shm_msg_metadata.inner_proc();
  QCHECK(metadata.void_addr());
  uint64 void_addr = metadata.void_addr();
  absl::MutexLock l(&mu_);
  RefInfo* ref_info = &FindOrDie(key_to_ref_info_, void_addr);
  ref_info->created_readers += 1;
  VLOG(4) << "AfterReceiveSharedMemoryMsg: required:"
          << ref_info->min_created_readers
          << " created: " << ref_info->created_readers;
}

void InnerProcessSharedMemoryManager::RegisterShmMsgSubscriber(
    const std::string& channel, const std::string& domain) {
  absl::WriterMutexLock l(&shm_reader_mu_);
  auto iter =
      shm_reader_channel_domain_cnt_.find(std::make_pair(channel, domain));
  if (iter == shm_reader_channel_domain_cnt_.end()) {
    shm_reader_channel_domain_cnt_[std::make_pair(channel, domain)] = 1;
  } else {
    iter->second++;
  }
}

// Create memory to read, if failed returns nullptr.
std::unique_ptr<SharedMemoryBlock>
InnerProcessSharedMemoryManager::CreateMemToRead(
    const ShmMessageMetadata& shm_msg_metadata) {
  ShmMessageMetadata new_metadata = shm_msg_metadata;
  new_metadata.set_reader_or_writer(true);
  const auto& metadata = new_metadata.inner_proc();
  if (metadata.void_addr() == 0) {
    return nullptr;
  }
  uint64 void_addr = metadata.void_addr();
  {
    absl::MutexLock l(&mu_);
    RefInfo* ref_info = FindOrNull(key_to_ref_info_, void_addr);
    if (ref_info == nullptr) {
      LOG(ERROR) << "Try to create a reader with non exists addr: " << void_addr
                 << ", metadata " << shm_msg_metadata.DebugString();
      return nullptr;
    }
    ref_info->ref_count++;
  }

  void* buffer = reinterpret_cast<void*>(void_addr);
  return std::make_unique<SharedMemoryBlock>(
      this, buffer, new_metadata.buffer_size(), new_metadata);
}

void InnerProcessSharedMemoryManager::ReleaseShmBlock(
    const ShmMessageMetadata& shm_msg_metadata) {
  const auto& metadata = shm_msg_metadata.inner_proc();
  uint64 void_addr = metadata.void_addr();
  QCHECK_GT(metadata.void_addr(), 0);
  absl::MutexLock l(&mu_);
  RefInfo* ref_info = &FindOrDie(key_to_ref_info_, void_addr);
  const std::string& channel = shm_msg_metadata.header().channel();

  // Reference count decrease to zero, should garbage collect it.
  if (--(ref_info->ref_count) == 0) {
    if (ref_info->created_readers < ref_info->min_created_readers) {
      VLOG(3) << "Number of create readers " << ref_info->created_readers
              << " is smaller than expected " << ref_info->min_created_readers
              << " won't garbage collect the addr " << void_addr;
      // TODO(kun): Having a another threads to check the really old buffer
      // blocks and garbage collect them. Since it's common a reader/writer
      // thread or process crash in the middle of program. We should ensure
      // orphan blocks get garbage collected eventually.

      return;
    }

    key_to_ref_info_.erase(void_addr);
    char* buffer = reinterpret_cast<char*>(void_addr);
    delete[] buffer;
    total_allocated_ram_ -= shm_msg_metadata.buffer_size();
    channel_to_allocated_ram_[channel] -= shm_msg_metadata.buffer_size();
  }
}

int32 InnerProcessSharedMemoryManager::NumOfAvaibleShmBlocks() {
  absl::ReaderMutexLock l(&mu_);
  return key_to_ref_info_.size();
}

int32 InnerProcessSharedMemoryManager::GetNumberOfRegisteredReaders(
    const std::string& channel, const std::string& domain) {
  absl::ReaderMutexLock l(&shm_reader_mu_);
  return FindWithDefault(shm_reader_channel_domain_cnt_,
                         std::make_pair(channel, domain), 0);
}

int64 InnerProcessSharedMemoryManager::GetUsedMemory() {
  absl::MutexLock l(&mu_);
  return total_allocated_ram_;
}

}  // namespace qcraft
