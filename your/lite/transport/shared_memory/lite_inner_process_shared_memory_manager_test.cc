#include "onboard/lite/transport/shared_memory/lite_inner_process_shared_memory_manager.h"

#include <iostream>
#include <memory>
#include <utility>

#include "gtest/gtest.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {

class LiteInnerMgmtTest : public ::testing::Test {
 protected:
  void SetUp() override {
    shm_manager_ = std::make_unique<InnerProcessSharedMemoryManager>();
  }

  void TearDown() override {}
  std::unique_ptr<InnerProcessSharedMemoryManager> shm_manager_;
  std::map<std::pair<std::string, std::string>, int32> registered_cnt_;
};

TEST_F(LiteInnerMgmtTest, AllocateMemToWrite_0) {
  ShmMsgType shm_msg_type = SHM_UNUSED;
  std::string channel = "decoded_image_1";

  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;

  std::unique_ptr<SharedMemoryBlock> shm_block;
  shm_block = shm_manager_->AllocateMemToWrite(options);
  EXPECT_NE(shm_block->buffer_, nullptr);

  EXPECT_NE(shm_manager_->NumOfAvaibleShmBlocks(), 0);
  shm_block.reset();
}

TEST_F(LiteInnerMgmtTest, AllocateMemToWrite_1) {
  ShmMsgType shm_msg_type = SHM_UNUSED;
  std::string channel = "decoded_image_1";

  SharedMemoryManager::Options options;
  options.buffer_size = 8LL << 30;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;
  EXPECT_DEATH(shm_manager_->AllocateMemToWrite(options), ".*");
}

TEST_F(LiteInnerMgmtTest, DumpAllocatedRamInfo) {
  ShmMsgType shm_msg_type = SHM_UNUSED;
  std::string channel = "decoded_image_1";

  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;
  std::unique_ptr<SharedMemoryBlock> shm_block;
  shm_block = shm_manager_->AllocateMemToWrite(options);
  EXPECT_NE(shm_block->buffer_, nullptr);

  SharedMemoryManager::Options options2;
  options2.buffer_size = 64;
  options2.shm_msg_type = shm_msg_type;
  options2.channel = channel;
  std::unique_ptr<SharedMemoryBlock> shm_block2;
  shm_block2 = shm_manager_->AllocateMemToWrite(options2);
  EXPECT_NE(shm_block2->buffer_, nullptr);

  InnerProcessSharedMemoryManager* manager =
      dynamic_cast<InnerProcessSharedMemoryManager*>(shm_manager_.get());
  absl::MutexLock l(&manager->mu_);
  EXPECT_EQ(manager->DumpAllocatedRamInfo(),
            "decoded_image_1 allocated 128 bytes\n");
}

TEST_F(LiteInnerMgmtTest, OtherFunc) {
  EXPECT_EQ(shm_manager_->NumOfAvaibleShmBlocks(), 0);
  EXPECT_EQ(shm_manager_->GetUsedMemory(), 0);
  EXPECT_EQ(shm_manager_->GetNumberOfRegisteredReaders("test_channel", ""), 0);
  shm_manager_->Reset();
  registered_cnt_[std::make_pair("test_shm", "")] = 2;
  shm_manager_->TEST_SetRegisteredShmMsgReaderCount(registered_cnt_);
}

TEST_F(LiteInnerMgmtTest, SendRcvMsg) {
  ShmMsgType shm_msg_type = SHM_UNUSED;
  std::string channel = "decoded_image_2";
  std::string domain = "";

  shm_manager_->RegisterShmMsgSubscriber(channel, domain);
  EXPECT_EQ(1, shm_manager_->GetNumberOfRegisteredReaders(channel, domain));
  shm_manager_->RegisterShmMsgSubscriber(channel, domain);
  EXPECT_EQ(2, shm_manager_->GetNumberOfRegisteredReaders(channel, domain));

  // send msg
  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;

  std::unique_ptr<SharedMemoryBlock> shm_block;
  shm_block = shm_manager_->AllocateMemToWrite(options);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_channel(channel);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_domain(domain);
  shm_block->MutableShmMsgMetadata()->set_shm_msg_type(shm_msg_type);
  EXPECT_NE(shm_block, nullptr);

  int min_reader = 0, reader = 0;
  uint64 void_addr = shm_block->ShmMsgMetadata().inner_proc().void_addr();
  InnerProcessSharedMemoryManager* manager =
      dynamic_cast<InnerProcessSharedMemoryManager*>(shm_manager_.get());
  manager->mu_.Lock();
  InnerProcessSharedMemoryManager::RefInfo* ref_info =
      &FindOrDie(manager->key_to_ref_info_, void_addr);
  min_reader = ref_info->min_created_readers;
  reader = ref_info->created_readers;
  manager->mu_.Unlock();

  shm_manager_->BeforeSendSharedMemoryMsg(shm_block->ShmMsgMetadata());
  shm_manager_->AfterReceiveSharedMemoryMsg(shm_block->ShmMsgMetadata());

  manager->mu_.Lock();
  EXPECT_EQ(ref_info->min_created_readers, min_reader + 2);
  EXPECT_EQ(ref_info->created_readers, reader + 1);
  manager->mu_.Unlock();

  std::unique_ptr<SharedMemoryBlock> shm_block2 =
      shm_manager_->CreateMemToRead(shm_block->ShmMsgMetadata());
  EXPECT_NE(shm_block2, nullptr);

  ShmMessageMetadata shm_msg_metadata;
  EXPECT_EQ(shm_manager_->CreateMemToRead(shm_msg_metadata), nullptr);

  int buffer_size = 1024;
  void* buffer = new char[buffer_size];
  void_addr = reinterpret_cast<uint64>(buffer);
  shm_msg_metadata.set_buffer_size(buffer_size);
  shm_msg_metadata.set_reader_or_writer(false);
  auto* metadata = shm_msg_metadata.mutable_inner_proc();
  metadata->set_void_addr(void_addr);
  EXPECT_EQ(shm_manager_->CreateMemToRead(shm_msg_metadata), nullptr);
}

}  // namespace qcraft
