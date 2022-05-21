#include "onboard/lite/transport/shm/shm_transmitter.h"

#include <iostream>
#include <memory>
#include <utility>

#include "absl/strings/str_cat.h"
#include "gtest/gtest.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager_factory.h"
#include "onboard/lite/transport/shared_memory/lite_shm_shared_memory_manager.h"
#include "onboard/lite/transport/shm/condition_notifier.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
namespace shm {

class ShmTransTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_transport_managed_shared_memory_size = 2000000;
    FLAGS_transport_managed_shared_memory_name =
        absl::StrCat("ShmTransmitter", gettid());
    ShmManager::Instance()->DestroyAndCreateShm();
  }

  void TearDown() override { ShmManager::Instance()->DestroyShm(); }
};

TEST_F(ShmTransTest, Enable) {
  std::unique_ptr<ShmTransmitter> transmitter_1 =
      std::make_unique<ShmTransmitter>(1);

  EXPECT_TRUE(transmitter_1->enabled_);
  transmitter_1->Enable();  // test this->enabled_ true
  transmitter_1->Disable();
  EXPECT_FALSE(transmitter_1->enabled_);
  transmitter_1->Enable();
  EXPECT_TRUE(transmitter_1->enabled_);
}

TEST_F(ShmTransTest, Transmit) {
  const uint64_t channel_id_ = 111;
  std::shared_ptr<PoseProto> pose = std::make_shared<PoseProto>();
  auto* header = pose->mutable_header();
  header->set_seq_number(1);
  header->set_module_id(channel_id_);
  const float yaw = 1.2;
  const float pitch = 1.3;
  const float roll = 1.4;
  pose->set_yaw(yaw);
  pose->set_pitch(pitch);
  pose->set_roll(roll);

  std::unique_ptr<ShmTransmitter> transmitter_1 =
      std::make_unique<ShmTransmitter>(1);
  transmitter_1->Disable();
  EXPECT_DEATH(transmitter_1->Transmit("/domain/channel", pose, 0), ".*");

  transmitter_1->Enable();
  EXPECT_TRUE(transmitter_1->Transmit("/domain/channel", pose, 0));

  transmitter_1->notifier_->Shutdown();
  EXPECT_FALSE(transmitter_1->Transmit("/domain/channel", pose, 1));
}

TEST_F(ShmTransTest, Msg) {
  std::unique_ptr<ShmTransmitter> transmitter =
      std::make_unique<ShmTransmitter>(1);
  const uint64_t channel_id_ = 111;
  std::shared_ptr<PoseProto> pose = std::make_shared<PoseProto>();
  auto* header = pose->mutable_header();
  header->set_seq_number(1);
  header->set_module_id(channel_id_);
  const float yaw = 1.2;
  const float pitch = 1.3;
  const float roll = 1.4;
  pose->set_yaw(yaw);
  pose->set_pitch(pitch);
  pose->set_roll(roll);

  EXPECT_TRUE(transmitter->Transmit("/domain/channel", pose, 1));
}

TEST_F(ShmTransTest, ShmMsg) {
  SharedMemoryManager* shm_manager = new ShmSharedMemoryManager();
  std::unique_ptr<ShmTransmitter> transmitter =
      std::make_unique<ShmTransmitter>(1);

  ShmMsgType shm_msg_type = SHM_DECODED_IMAGE;
  std::string channel = "decoded_image";
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
  EXPECT_EQ(subscriber_num, 1);

  ShmManager::Instance()->ShowDebug();
  shm_block = shm_manager->AllocateMemToWrite(options);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_channel(channel);
  shm_block->MutableShmMsgMetadata()->mutable_header()->set_domain(domain);
  shm_block->MutableShmMsgMetadata()->set_shm_msg_type(shm_msg_type);
  EXPECT_NE(shm_block, nullptr);

  shm_manager->BeforeSendSharedMemoryMsg(shm_block->ShmMsgMetadata());

  std::shared_ptr<google::protobuf::Message> shared_msg;
  shared_msg.reset(shm_block->ShmMsgMetadata().New());
  shared_msg->CopyFrom(shm_block->ShmMsgMetadata());

  EXPECT_TRUE(
      transmitter->Transmit(domain_channel, shared_msg, subscriber_num));

  shm_block.reset();
  ShmManager::Instance()->UnSubscribe(domain_channel, subscriber_num);
}
}  // namespace shm
}  // namespace qcraft
