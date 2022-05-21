#include "onboard/lite/lite_shm_message.h"

#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/proto/lite_common.pb.h"

namespace qcraft {
namespace {

class LiteShmMessageTest : public testing::Test {
 protected:
  void SetUp() override { ShmFactory::GetShmManager()->Reset(); }

  struct SimpleStruct {
    int size;
    std::array<double, 10> numbers;
    std::array<char, 10> chars;
    REGISTER_SHM_MSG(SimpleStruct)
  };

  void Print(const SimpleStruct& stru1) {
    std::cout << "size = " << stru1.size << std::endl;
    for (int i = 0; i < 10; i++) {
      std::cout << stru1.numbers[i] << ",";
    }
    std::cout << std::endl;
    for (int i = 0; i < 10; i++) {
      std::cout << stru1.chars[i] << ",";
    }
    std::cout << std::endl;
  }

  struct ComplexStruct {
    int size;
    std::array<double, 10> numbers;
    // pointer to string with length 26.
    char* str;

    static int32 ExpectedSize() { return sizeof(ComplexStruct) + 27; }

    static ComplexStruct* CastToPointer(void* buffer) {
      ComplexStruct* s = static_cast<ComplexStruct*>(buffer);
      char* p = static_cast<char*>(buffer) + sizeof(ComplexStruct);
      s->str = p;
      return s;
    }

    static const ComplexStruct& CastToReference(const void* buffer) {
      return *CastToPointer(const_cast<ComplexStruct*>(
          (static_cast<const ComplexStruct*>(buffer))));
    }
  };

  void Print(const ComplexStruct& stru1) {
    std::cout << "size = " << stru1.size << std::endl;
    for (int i = 0; i < 10; i++) {
      std::cout << stru1.numbers[i] << ",";
    }
    std::cout << std::endl;
    std::cout << "str= " << stru1.str << std::endl;
  }
  std::map<std::pair<std::string, std::string>, int32> registered_cnt_;
};

TEST_F(LiteShmMessageTest, SimpleWriteAndRead) {
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  ShmFactory::GetShmManager()->RegisterShmMsgSubscriber("test_shm", "");
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "test_shm");
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  ShmFactory::GetShmManager()->BeforeSendSharedMemoryMsg(
      shm_stru1->shm_msg_metadata());
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  Print(shm_stru1->value<SimpleStruct>());
  ShmMessageMetadata metadata = shm_stru1->shm_msg_metadata();
  EXPECT_EQ(metadata.shm_msg_type(), SHM_UNKNOWN);
  shm_stru1.reset();
  // Won't delete the block since no reader read the block yet.
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  std::shared_ptr<ShmMessage> shm_stru2 = ShmMessage::CreateToRead(metadata);
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(metadata);
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  Print(shm_stru2->value<SimpleStruct>());
  EXPECT_EQ(shm_stru2->value<SimpleStruct>().size, 10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(shm_stru2->value<SimpleStruct>().numbers[i],
              static_cast<double>(i));
    EXPECT_EQ(shm_stru2->value<SimpleStruct>().chars[i], 'a' + i);
  }
  auto shm_stru3 = shm_stru2;
  shm_stru2.reset();
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  shm_stru3.reset();
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  EXPECT_EQ(ShmMessage::CreateToRead(metadata), nullptr);
}

TEST_F(LiteShmMessageTest, RegisterSubscriberTwice) {
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  ShmFactory::GetShmManager()->RegisterShmMsgSubscriber("test_shm", "");
  ShmFactory::GetShmManager()->RegisterShmMsgSubscriber("test_shm", "");
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "test_shm");
  ShmFactory::GetShmManager()->BeforeSendSharedMemoryMsg(
      shm_stru1->shm_msg_metadata());
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  Print(shm_stru1->value<SimpleStruct>());
  ShmMessageMetadata metadata = shm_stru1->shm_msg_metadata();
  EXPECT_EQ(metadata.shm_msg_type(), SHM_UNKNOWN);
  shm_stru1.reset();
  // Won't delete the block since no reader read the block yet.
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  std::shared_ptr<ShmMessage> shm_stru2 = ShmMessage::CreateToRead(metadata);

  // simulate 2 subscriber.
  ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(metadata);
  ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(metadata);

  Print(shm_stru2->value<SimpleStruct>());
  EXPECT_EQ(shm_stru2->value<SimpleStruct>().size, 10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(shm_stru2->value<SimpleStruct>().numbers[i],
              static_cast<double>(i));
    EXPECT_EQ(shm_stru2->value<SimpleStruct>().chars[i], 'a' + i);
  }
  auto shm_stru3 = shm_stru2;
  shm_stru2.reset();
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  shm_stru3.reset();
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  EXPECT_EQ(ShmMessage::CreateToRead(metadata), nullptr);
}

TEST_F(LiteShmMessageTest, OneWriteAndMultiRead) {
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  registered_cnt_[std::make_pair("test_shm", "")] = 2;
  ShmFactory::GetShmManager()->TEST_SetRegisteredShmMsgReaderCount(
      registered_cnt_);
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "test_shm");

  ShmFactory::GetShmManager()->BeforeSendSharedMemoryMsg(
      shm_stru1->shm_msg_metadata());

  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  ShmMessageMetadata metadata = shm_stru1->shm_msg_metadata();
  shm_stru1.reset();
  // Won't delete the block since no reader read the block yet.
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  std::shared_ptr<ShmMessage> shm_stru2 = ShmMessage::CreateToRead(metadata);
  // 2 reader, first reader
  ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(metadata);
  Print(shm_stru2->value<SimpleStruct>());
  EXPECT_EQ(shm_stru2->value<SimpleStruct>().size, 10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(shm_stru2->value<SimpleStruct>().numbers[i],
              static_cast<double>(i));
    EXPECT_EQ(shm_stru2->value<SimpleStruct>().chars[i], 'a' + i);
  }
  shm_stru2.reset();
  // Only one reader created, won't be released.
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  std::vector<std::shared_ptr<ShmMessage>> readers;
  for (int j = 0; j < 7; j++) {
    readers.emplace_back(ShmMessage::CreateToRead(metadata));
    const SimpleStruct& v = readers.back()->value<SimpleStruct>();
    EXPECT_EQ(v.size, 10);
    for (int i = 0; i < 10; i++) {
      EXPECT_EQ(v.numbers[i], static_cast<double>(i));
      EXPECT_EQ(v.chars[i], 'a' + i);
    }
    EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  }
  ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(metadata);
  readers.clear();
  // second reader
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
}

TEST_F(LiteShmMessageTest, ComplexStructTest) {
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  ShmFactory::GetShmManager()->RegisterShmMsgSubscriber("test_shm", "");
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<ComplexStruct>(SHM_UNKNOWN, "test_shm");
  ShmFactory::GetShmManager()->BeforeSendSharedMemoryMsg(
      shm_stru1->shm_msg_metadata());
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  ComplexStruct* v = shm_stru1->mutable_value<ComplexStruct>();
  v->size = 10;
  for (int i = 0; i < 10; i++) {
    v->numbers[i] = static_cast<double>(i);
  }
  const std::string alphabet = "abcdefghijklmnopqrstuvwxyz";
  for (int i = 0; i < 26; i++) {
    *(v->str + i) = alphabet[i];
  }
  *(v->str + 26) = '\0';
  Print(shm_stru1->value<ComplexStruct>());

  ShmMessageMetadata metadata = shm_stru1->shm_msg_metadata();
  shm_stru1.reset();
  // Won't delete the block since no reader read the block yet.
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  std::shared_ptr<ShmMessage> shm_stru2 = ShmMessage::CreateToRead(metadata);
  Print(shm_stru2->value<ComplexStruct>());
  EXPECT_EQ(shm_stru2->value<ComplexStruct>().size, 10);
  for (int i = 0; i < 10; i++) {
    EXPECT_EQ(shm_stru2->value<ComplexStruct>().numbers[i],
              static_cast<double>(i));
  }
  EXPECT_EQ(alphabet, std::string(shm_stru2->value<ComplexStruct>().str));
  ShmFactory::GetShmManager()->AfterReceiveSharedMemoryMsg(metadata);
  shm_stru2.reset();
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  EXPECT_EQ(ShmMessage::CreateToRead(metadata), nullptr);
}

}  // namespace
}  // namespace qcraft
