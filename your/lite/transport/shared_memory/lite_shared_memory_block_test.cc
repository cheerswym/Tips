#include "onboard/lite/transport/shared_memory/lite_shared_memory_block.h"

#include "gtest/gtest.h"
#include "onboard/lite/transport/shared_memory/lite_inner_process_shared_memory_manager.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager.h"

namespace qcraft {
namespace shm {

void SharedMemoryBlockTestBase(bool flag) {
  std::unique_ptr<InnerProcessSharedMemoryManager> shm_manager;
  shm_manager = std::make_unique<InnerProcessSharedMemoryManager>();
  int32_t length = 100;
  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = SHM_DECODED_IMAGE;
  options.channel = "test_channel";
  void* buffer = new char[length];
  uint64 void_addr = reinterpret_cast<uint64>(buffer);

  ShmMessageMetadata shm_msg_metadata;
  shm_msg_metadata.set_buffer_size(length);
  shm_msg_metadata.set_reader_or_writer(flag);
  auto* metadata = shm_msg_metadata.mutable_inner_proc();
  metadata->set_void_addr(void_addr);

  InnerProcessSharedMemoryManager* manager =
      dynamic_cast<InnerProcessSharedMemoryManager*>(shm_manager.get());
  manager->mu_.Lock();
  QCHECK(!ContainsKey(manager->key_to_ref_info_, void_addr));
  InnerProcessSharedMemoryManager::RefInfo* ref_info =
      &(manager->key_to_ref_info_[void_addr]);
  ref_info->ref_count = 1;
  ref_info->created_readers = 0;
  ref_info->min_created_readers = 0;
  ref_info->options = options;
  manager->mu_.Unlock();

  qcraft::SharedMemoryBlock shm_block(shm_manager.get(), buffer, length,
                                      shm_msg_metadata);
  if (flag) {
    EXPECT_EQ(shm_block.MutableBuffer(), nullptr);
  } else {
    EXPECT_NE(shm_block.MutableBuffer(), nullptr);
  }
  EXPECT_NE(shm_block.Buffer(), nullptr);
  EXPECT_EQ(shm_block.BufferSize(), length);
}

TEST(SharedMemoryBlockTest, MutableBuffer_TRUE) {
  SharedMemoryBlockTestBase(true);
}

TEST(SharedMemoryBlockTest, MutableBuffer_FALSE) {
  SharedMemoryBlockTestBase(false);
}

}  // namespace shm
}  // namespace qcraft
