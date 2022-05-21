#include "onboard/lite/transport/shm/shm_manager.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/transport/shm/condition_notifier.h"
#include "onboard/lite/transport/shm/lite_shm_header.h"

namespace qcraft {
namespace shm {

class ShmManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_transport_managed_shared_memory_size = 2000000;
    FLAGS_transport_managed_shared_memory_name =
        absl::StrCat("ShmManagerTest", gettid());
    ShmManager::Instance()->DestroyAndCreateShm();
  }

  void TearDown() override { ShmManager::Instance()->DestroyShm(); }
};

TEST_F(ShmManagerTest, Initialization) {
  EXPECT_EQ(ShmManager::Instance()->GetMaxMemory(),
            FLAGS_transport_managed_shared_memory_size);
  EXPECT_LE(ShmManager::Instance()->GetFreeMemory(),
            FLAGS_transport_managed_shared_memory_size);
  auto* indicator1 = ShmManager::Instance()->CreateNotifierIndicator();
  auto* indicator2 = ShmManager::Instance()->CreateNotifierIndicator();
  EXPECT_NE(indicator1, nullptr);
  EXPECT_NE(indicator2, nullptr);

  EXPECT_LE(ShmManager::Instance()->GetUsedMemory(),
            ShmManager::Instance()->GetMaxMemory());
  EXPECT_LE(0, ShmManager::Instance()->GetUsedMemory());

  indicator1->next_write_seq = 100;
  EXPECT_EQ(indicator2->next_write_seq, 100);
}

TEST_F(ShmManagerTest, AttachShm) {
  ShmManager::Instance()->AttachShm();
  EXPECT_EQ(ShmManager::Instance()->GetMaxMemory(),
            FLAGS_transport_managed_shared_memory_size);
  EXPECT_LE(ShmManager::Instance()->GetFreeMemory(),
            FLAGS_transport_managed_shared_memory_size);
  ShmManager::Instance()->DestroyShm();
}

TEST_F(ShmManagerTest, Allocate) {
  void* mem = ShmManager::Instance()->Allocate(100);
  EXPECT_NE(mem, nullptr);
  auto handle = ShmManager::Instance()->GetHandleFromAddress(mem);
  ShmManager::Instance()->IncreaseRefCount(handle, 3);
  EXPECT_EQ(ShmManager::Instance()->GetRefCount(handle), 3);

  ShmManager::Instance()->DecreaseRefCount(handle, 1);
  EXPECT_EQ(ShmManager::Instance()->GetRefCount(handle), 2);

  ShmManager::Instance()->DecreaseRefCount(handle, 1);
  EXPECT_EQ(ShmManager::Instance()->GetRefCount(handle), 1);

  ShmManager::Instance()->DecreaseRefCount(handle, 1);
  EXPECT_EQ(ShmManager::Instance()->GetRefCount(handle), 0);
}

TEST_F(ShmManagerTest, AccessDeallocated) {
  void* mem = ShmManager::Instance()->Allocate(100);
  EXPECT_NE(mem, nullptr);
  ShmManager::Instance()->Deallocate(mem);
}

TEST_F(ShmManagerTest, ReadWrite) {
  int32_t cnt = 10;
  int subject = 1;

  ShmManager::Instance()->ShowDebug();

  auto notifier = std::make_unique<shm::ConditionNotifier>();
  for (int i = 0; i < cnt; ++i) {
    uint64_t current_write_seq = 0, current_write_index = 0;
    CHECK(notifier->GetWriteIndex(&current_write_seq, &current_write_index));
    auto address = qcraft::shm::ShmManager::Instance()->Allocate(100);
    auto handle =
        qcraft::shm::ShmManager::Instance()->GetHandleFromAddress(address);
    qcraft::shm::ShmManager::Instance()->IncreaseRefCount(handle, 1);
    EXPECT_EQ(
        shm::ShmManager::Instance()->GetIndexRefCount(current_write_index), 0);
    EXPECT_EQ(shm::ShmManager::Instance()->GetRefCount(handle), 1);
    size_t header_size = 0;
    size_t msg_size = 0;
    size_t domain_channel_hash = subject;

    shm::ReadableInfo readable_info{handle, header_size, msg_size,
                                    domain_channel_hash};

    CHECK(notifier->Write(readable_info, 1, current_write_seq,
                          current_write_index));
    EXPECT_EQ(
        shm::ShmManager::Instance()->GetIndexRefCount(current_write_index), 1);
    EXPECT_EQ(shm::ShmManager::Instance()->GetRefCount(readable_info.handle),
              1);
    usleep(10000);
  }

  ShmManager::Instance()->ShowDebug();
  while (cnt > 0) {
    shm::ReadableInfo readable_info;
    int index = 0;
    if (!notifier->Read(100, &readable_info, &index)) {
      continue;
    }
    if (readable_info.domain_channel_hash != subject) {
      continue;
    }
    cnt--;
    EXPECT_NE(
        shm::ShmManager::Instance()->GetAddressFromHandle(readable_info.handle),
        nullptr);
    shm::ShmManager::Instance()->DecreaseRefCount(readable_info.handle, 1);
    shm::ShmManager::Instance()->DecreaseIndexRefCount(index, 1);

    EXPECT_EQ(shm::ShmManager::Instance()->GetRefCount(readable_info.handle),
              0);
    EXPECT_EQ(shm::ShmManager::Instance()->GetIndexRefCount(index), 0);
    ShmManager::Instance()->GarbageCollection(index);
  }
}

TEST_F(ShmManagerTest, SegReset) {
  ShmManager::Instance()->segment_.reset();
  auto* indicator = ShmManager::Instance()->CreateNotifierIndicator();
  EXPECT_NE(indicator, nullptr);

  ShmManager::Instance()->segment_.reset();
  void* mem = ShmManager::Instance()->Allocate(100);
  EXPECT_NE(mem, nullptr);

  ShmManager::Instance()->segment_.reset();
  auto handle = ShmManager::Instance()->GetHandleFromAddress(mem);
  ShmManager::Instance()->segment_.reset();
  mem = shm::ShmManager::Instance()->GetAddressFromHandle(handle);
  EXPECT_NE(mem, nullptr);
}

TEST_F(ShmManagerTest, Subscribe) {
  std::string channel = "decoded_image_shm";
  std::string domain = "";

  std::string domain_channel = CombineDomainChannel(domain, channel);
  ShmManager::Instance()->Subscribe(domain_channel);
  int subscriber_num =
      shm::ShmManager::Instance()->GetSubscriberNum(domain_channel);
  EXPECT_EQ(subscriber_num, 1);
  ShmManager::Instance()->UnSubscribe(domain_channel, subscriber_num);
  subscriber_num =
      shm::ShmManager::Instance()->GetSubscriberNum(domain_channel);
  EXPECT_EQ(subscriber_num, 0);
  subscriber_num = shm::ShmManager::Instance()->GetSubscriberNum("tests");
  EXPECT_EQ(subscriber_num, 0);
}

TEST_F(ShmManagerTest, IndexZero) {
  shm::ShmManager::Instance()->DecreaseIndexRefCount(1, 0);
  shm::ShmManager::Instance()->IncreaseIndexRefCount(1, 0);

  void* mem = ShmManager::Instance()->Allocate(100);
  EXPECT_NE(mem, nullptr);
  auto handle = ShmManager::Instance()->GetHandleFromAddress(mem);
  ShmManager::Instance()->IncreaseRefCount(handle, 0);

  ShmManager::Instance()->DecreaseRefCount(handle, 0);
  ShmManager::Instance()->GetRefCount(handle);
  ShmManager::Instance()->DecreaseRefCount(handle, 1);
}

TEST_F(ShmManagerTest, GarbageCollection) {
  ShmManager::Instance()->GarbageCollection(1);

  auto notifier = std::make_unique<shm::ConditionNotifier>();
  for (int i = 0; i < 10; ++i) {
    LiteHeader header;
    header.set_seq_number(123);
    header.set_module_id(2);
    header.set_timestamp(2000);
    // kShmTagNumber
    LiteShmHeader lite_shm_header("test_channel", 6, header);

    uint64_t current_write_seq = 0, current_write_index = 0;
    CHECK(notifier->GetWriteIndex(&current_write_seq, &current_write_index));
    auto address = qcraft::shm::ShmManager::Instance()->Allocate(100);
    auto handle =
        qcraft::shm::ShmManager::Instance()->GetHandleFromAddress(address);
    EXPECT_TRUE(lite_shm_header.SerializeTo(reinterpret_cast<char*>(address)));
    qcraft::shm::ShmManager::Instance()->IncreaseRefCount(handle, 1);
    EXPECT_EQ(
        shm::ShmManager::Instance()->GetIndexRefCount(current_write_index), 0);
    EXPECT_EQ(shm::ShmManager::Instance()->GetRefCount(handle), 1);
    size_t header_size = 0;
    size_t msg_size = 0;
    size_t domain_channel_hash = 1;

    shm::ReadableInfo readable_info{handle, header_size, msg_size,
                                    domain_channel_hash};

    CHECK(notifier->Write(readable_info, 1, current_write_seq,
                          current_write_index));
    EXPECT_EQ(
        shm::ShmManager::Instance()->GetIndexRefCount(current_write_index), 1);
    EXPECT_EQ(shm::ShmManager::Instance()->GetRefCount(readable_info.handle),
              1);
    shm::ShmManager::Instance()->IncreaseIndexRefCount(current_write_index, 1);
    ShmManager::Instance()->GarbageCollection(current_write_index);
    usleep(10000);
  }
}

}  // namespace shm
}  // namespace qcraft
