#include "onboard/lite/transport/shared_memory/lite_shm_shared_memory_manager.h"

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
#include "onboard/global/counter.h"
#include "onboard/lite/lite_message_util.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/proto/shm_message.pb.h"
#include "onboard/lite/transport/shm/shm_manager.h"
#include "onboard/utils/map_util.h"

namespace qcraft {

std::unique_ptr<SharedMemoryBlock> ShmSharedMemoryManager::AllocateMemToWrite(
    const SharedMemoryManager::Options& options) {
  absl::MutexLock l(&mu_);
  void* void_addr = shm::ShmManager::Instance()->Allocate(options.buffer_size);
  if (void_addr == nullptr) {
    LOG(FATAL) << "failed to allocate on shm";
    return nullptr;
  }
  size_t total_allocated_ram = shm::ShmManager::Instance()->GetUsedMemory();
  QCOUNTER("allocated_shm_mb", total_allocated_ram >> 20);
  ShmMessageMetadata shm_msg_metadata;
  shm_msg_metadata.set_buffer_size(options.buffer_size);
  shm_msg_metadata.set_reader_or_writer(false);

  auto* metadata = shm_msg_metadata.mutable_cross_proc();
  auto handle = shm::ShmManager::Instance()->GetHandleFromAddress(void_addr);
  metadata->set_handle(handle);
  shm::ShmManager::Instance()->IncreaseRefCount(handle, 1);

  return std::make_unique<SharedMemoryBlock>(
      this, void_addr, options.buffer_size, shm_msg_metadata);
}

void ShmSharedMemoryManager::BeforeSendSharedMemoryMsg(
    const ShmMessageMetadata& shm_message_proto) {
  VLOG(3) << "ShmSharedMemoryManager::BeforeSendSharedMemoryMsg";
  std::string domain_channel =
      CombineDomainChannel(shm_message_proto.header().domain(),
                           shm_message_proto.header().channel());

  const int sub_num =
      shm::ShmManager::Instance()->GetSubscriberNum(domain_channel);
  shm::ShmManager::Instance()->IncreaseRefCount(
      shm_message_proto.cross_proc().handle(), sub_num);
}

void ShmSharedMemoryManager::AfterReceiveSharedMemoryMsg(
    const ShmMessageMetadata& shm_message_proto) {
  VLOG(3) << "ShmSharedMemoryManager::AfterReceiveSharedMemoryMsg";
  shm::ShmManager::Instance()->DecreaseRefCount(
      shm_message_proto.cross_proc().handle(), 1);
}

void ShmSharedMemoryManager::RegisterShmMsgSubscriber(
    const std::string& channel, const std::string& domain) {
  // We already know how many subscriber, and we bump the ref count with the
  // handle name, see above.
}

std::unique_ptr<SharedMemoryBlock> ShmSharedMemoryManager::CreateMemToRead(
    const ShmMessageMetadata& shm_msg_metadata) {
  VLOG(3) << "CreateMemToRead";
  ShmMessageMetadata new_metadata = shm_msg_metadata;
  new_metadata.set_reader_or_writer(true);
  const auto handle = new_metadata.cross_proc().handle();

  // Whether the handle has be recycled
  auto& handle_in_queue = shm::ShmManager::Instance()
                              ->CreateNotifierIndicator()
                              ->infos[shm_msg_metadata.cross_proc().index()]
                              .handle;
  if (shm_msg_metadata.cross_proc().msg_handle() != handle_in_queue) {
    LOG(ERROR) << "!!! Handle:" << shm_msg_metadata.cross_proc().handle()
               << " has be replaced by new handle: " << handle_in_queue;
    return nullptr;
  }

  shm::ShmManager::Instance()->IncreaseRefCount(handle, 1);

  void* void_addr = shm::ShmManager::Instance()->GetAddressFromHandle(handle);
  return std::make_unique<SharedMemoryBlock>(
      this, void_addr, new_metadata.buffer_size(), new_metadata);
}

void ShmSharedMemoryManager::ReleaseShmBlock(
    const ShmMessageMetadata& shm_msg_metadata) {
  VLOG(3) << "ReleaseShmBlock";
  if (shm_msg_metadata.cross_proc().has_index()) {
    shm::ShmManager::Instance()->DecreaseIndexRefCountOfSameHandle(
        shm_msg_metadata, 1);
  } else {
    shm::ShmManager::Instance()->DecreaseRefCount(
        shm_msg_metadata.cross_proc().handle(), 1);
  }
}

int32 ShmSharedMemoryManager::NumOfAvaibleShmBlocks() { return 0; }

int32 ShmSharedMemoryManager::GetNumberOfRegisteredReaders(
    const std::string& channel, const std::string& domain) {
  const std::string domain_channel = CombineDomainChannel(domain, channel);
  int sub_num = shm::ShmManager::Instance()->GetSubscriberNum(domain_channel);
  VLOG(3) << "GetNumberOfRegisteredReaders:" << sub_num;
  return sub_num;
}

void ShmSharedMemoryManager::Reset() {
  // In the shm conecpt, it's tricky to reset the memory (since everyone
  // using it), we recreate and clean the memory everytime we start the run.
  // see shm_manager.
}

void ShmSharedMemoryManager::TEST_SetRegisteredShmMsgReaderCount(
    const std::map<std::pair<std::string, std::string>, int32>&
        registered_reader_channel_domain_cnt) {}

int64_t ShmSharedMemoryManager::GetUsedMemory() {
  return shm::ShmManager::Instance()->GetUsedMemory();
}

}  // namespace qcraft
