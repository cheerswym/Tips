#include "onboard/lite/transport/shared_memory/lite_shm_shared_memory_manager.h"

#include <iostream>
#include <memory>
#include <utility>

#include "gtest/gtest.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"
#include "onboard/lite/transport/shm/shm_manager.h"
#include "onboard/lite/transport/shm/shm_transmitter.h"

namespace qcraft {
namespace shm {

class LiteShmMgmtTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_transport_managed_shared_memory_size = 2000000;
    FLAGS_transport_managed_shared_memory_name = "LiteShmMgmtTest";
    ShmManager::Instance()->DestroyAndCreateShm();
    shm_manager_ = std::make_unique<ShmSharedMemoryManager>();
  }

  void TearDown() override { ShmManager::Instance()->DestroyShm(); }
  std::unique_ptr<ShmSharedMemoryManager> shm_manager_;
};

TEST_F(LiteShmMgmtTest, AllocateMemToWrite) {
  ShmMsgType shm_msg_type = SHM_DECODED_IMAGE;
  std::string channel = "decoded_image_1";

  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;

  std::unique_ptr<SharedMemoryBlock> shm_block;

  shm_block = shm_manager_->AllocateMemToWrite(options);
  EXPECT_NE(shm_block->buffer_, nullptr);

  shm_block.reset();
}

TEST_F(LiteShmMgmtTest, SendRcvShmMsg) {
  std::unique_ptr<ShmTransmitter> transmitter =
      std::make_unique<ShmTransmitter>(1);
  ShmMsgType shm_msg_type = SHM_DECODED_IMAGE;
  std::string channel = "decoded_image_2";
  std::string domain = "";
  std::string domain_channel = CombineDomainChannel(domain, channel);

  // send msg
  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;

  std::unique_ptr<SharedMemoryBlock> shm_block;

  ShmManager::Instance()->Subscribe(domain_channel);
  int subscriber_num =
      shm::ShmManager::Instance()->GetSubscriberNum(domain_channel);
  EXPECT_EQ(1, subscriber_num);
  EXPECT_EQ(1, shm_manager_->GetNumberOfRegisteredReaders(channel, domain));

  shm_block = shm_manager_->AllocateMemToWrite(options);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_channel(channel);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_domain(domain);
  shm_block->MutableShmMsgMetadata()->set_shm_msg_type(shm_msg_type);
  EXPECT_NE(shm_block, nullptr);

  int ref = shm::ShmManager::Instance()->GetRefCount(
      shm_block->ShmMsgMetadata().cross_proc().handle());
  EXPECT_EQ(ref, 1);
  shm_manager_->BeforeSendSharedMemoryMsg(shm_block->ShmMsgMetadata());
  ref = shm::ShmManager::Instance()->GetRefCount(
      shm_block->ShmMsgMetadata().cross_proc().handle());
  EXPECT_EQ(ref, 2);
  shm_manager_->AfterReceiveSharedMemoryMsg(shm_block->ShmMsgMetadata());
  ref = shm::ShmManager::Instance()->GetRefCount(
      shm_block->ShmMsgMetadata().cross_proc().handle());
  EXPECT_EQ(ref, 1);
  ShmManager::Instance()->UnSubscribe(domain_channel, subscriber_num);
}

TEST_F(LiteShmMgmtTest, ReleaseShmBlock) {
  ShmMsgType shm_msg_type = SHM_DECODED_IMAGE;
  std::string channel = "decoded_image_3";
  std::string domain = "";

  SharedMemoryManager::Options options;
  options.buffer_size = 64;
  options.shm_msg_type = shm_msg_type;
  options.channel = channel;

  std::unique_ptr<SharedMemoryBlock> shm_block, shm_block_index;

  shm_block = shm_manager_->AllocateMemToWrite(options);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_channel(channel);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_domain(domain);
  shm_block->MutableShmMsgMetadata()->set_shm_msg_type(shm_msg_type);
  EXPECT_NE(shm_block, nullptr);

  shm_manager_->BeforeSendSharedMemoryMsg(shm_block->ShmMsgMetadata());
  shm_manager_->BeforeSendSharedMemoryMsg(shm_block->ShmMsgMetadata());

  std::unique_ptr<SharedMemoryBlock> shm_block_2 =
      shm_manager_->CreateMemToRead(shm_block->ShmMsgMetadata());
  EXPECT_NE(shm_block_2, nullptr);

  shm_manager_->ReleaseShmBlock(shm_block->ShmMsgMetadata());
  shm_manager_->ReleaseShmBlock(shm_block_2->ShmMsgMetadata());

  shm_block_index = shm_manager_->AllocateMemToWrite(options);
  shm_block_index->MutableShmMsgMetadata()->mutable_header()->set_channel(
      channel);
  shm_block_index->MutableShmMsgMetadata()->mutable_header()->set_domain(
      domain);
  shm_block_index->MutableShmMsgMetadata()->set_shm_msg_type(shm_msg_type);
  shm_block_index->MutableShmMsgMetadata()->mutable_cross_proc()->set_index(1);
  EXPECT_NE(shm_block_index, nullptr);
  shm_manager_->ReleaseShmBlock(shm_block_index->ShmMsgMetadata());
}

TEST_F(LiteShmMgmtTest, OtherFunc) {
  std::map<std::pair<std::string, std::string>, int32> map;
  map[std::make_pair("test_map", "")] = 1;
  shm_manager_->TEST_SetRegisteredShmMsgReaderCount(map);
  shm_manager_->RegisterShmMsgSubscriber("", "");

  EXPECT_EQ(shm_manager_->NumOfAvaibleShmBlocks(), 0);
  EXPECT_GE(shm_manager_->GetUsedMemory(), 0);
  shm_manager_->Reset();
}

TEST_F(LiteShmMgmtTest, NoMem) {
  ShmMsgType shm_msg_type = SHM_DECODED_IMAGE;
  std::string channel = "decoded_image_1";
  std::string domain = "";
  std::string domain_channel = CombineDomainChannel(domain, channel);

  SharedMemoryManager::Options options;
  options.buffer_size = FLAGS_transport_managed_shared_memory_size;
  options.shm_msg_type = shm_msg_type;
  options.channel = domain_channel;

  EXPECT_DEATH(shm_manager_->AllocateMemToWrite(options), ".*");
}

}  // namespace shm
}  // namespace qcraft
