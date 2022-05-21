#include "onboard/lite/transport/shm/condition_notifier.h"

#include <thread>

#include "gtest/gtest.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
namespace shm {

class CondNotifierTest : public ::testing::Test {
 protected:
  void SetUp() override {
    FLAGS_transport_managed_shared_memory_size = 2000000;
    FLAGS_transport_managed_shared_memory_name =
        absl::StrCat("CondNotifierTest", gettid());
    ShmManager::Instance()->DestroyAndCreateShm();
  }

  void TearDown() override { ShmManager::Instance()->DestroyShm(); }
};

TEST_F(CondNotifierTest, ShutDown) {
  ReadableInfo readable_info;
  uint64_t write_seq = 0, write_index = 0;
  int index = 0;

  auto notifier = ConditionNotifier();
  EXPECT_TRUE(notifier.GetWriteIndex(&write_seq, &write_index));
  EXPECT_EQ(write_seq + 1, notifier.indicator_->next_write_seq);

  notifier.Shutdown();
  EXPECT_FALSE(notifier.GetWriteIndex(&write_seq, &write_index));
  EXPECT_FALSE(notifier.Read(10, &readable_info, &index));
}

TEST_F(CondNotifierTest, ReaderCatchup) {
  uint64_t write_seq = 0, write_index = 0;

  auto notifier = ConditionNotifier();
  EXPECT_TRUE(notifier.GetWriteIndex(&write_seq, &write_index));
  notifier.ReaderCatchup();
  EXPECT_EQ(write_seq, notifier.my_read_seq_);
}

TEST_F(CondNotifierTest, notify_listen) {
  ReadableInfo readable_info;
  int index = 0;
  auto notifier = ConditionNotifier();
  while (notifier.Read(10, &readable_info, &index)) {
  }
  EXPECT_FALSE(notifier.Read(1, &readable_info, &index));
  EXPECT_TRUE(notifier.Write(readable_info, 1, 1, 1));
  EXPECT_TRUE(notifier.Read(1, &readable_info, &index));
  EXPECT_FALSE(notifier.Read(1, &readable_info, &index));
  EXPECT_TRUE(notifier.Write(readable_info, 1, 2, 2));
  EXPECT_TRUE(notifier.Write(readable_info, 1, 3, 3));
  EXPECT_TRUE(notifier.Read(1, &readable_info, &index));
  EXPECT_TRUE(notifier.Read(1, &readable_info, &index));
  EXPECT_FALSE(notifier.Read(1, &readable_info, &index));
}

TEST_F(CondNotifierTest, multithread_2_writer_1_reader) {
  auto writer1_notifier = ConditionNotifier();
  auto writer2_notifier = ConditionNotifier();
  auto reader_notifier = ConditionNotifier();
  std::thread writer1([&writer1_notifier] {
    for (int i = 0; i < 100; ++i) {
      ReadableInfo readable_info;
      readable_info.header_size = i;
      writer1_notifier.Write(readable_info, 1, i, i);
    }
  });

  std::thread writer2([&writer2_notifier] {
    for (int i = 100; i < 200; ++i) {
      ReadableInfo readable_info;
      readable_info.header_size = i;
      writer2_notifier.Write(readable_info, 1, i, i);
    }
  });
  writer1.join();
  writer2.join();

  size_t counter = 0;
  for (int i = 0; i < 200; ++i) {
    ReadableInfo readable_info;
    int index;
    reader_notifier.Read(1, &readable_info, &index);
    counter += readable_info.header_size;
  }
  EXPECT_EQ(counter, 19900);
}

TEST_F(CondNotifierTest, multithread_2_writer_2_reader) {
  auto writer1_notifier = ConditionNotifier();
  auto writer2_notifier = ConditionNotifier();
  auto reader1_notifier = ConditionNotifier();
  auto reader2_notifier = ConditionNotifier();

  std::thread writer1([&writer1_notifier] {
    for (int i = 0; i < 100; ++i) {
      ReadableInfo readable_info;
      readable_info.header_size = i;
      writer1_notifier.Write(readable_info, 1, i, i);
    }
  });

  std::thread writer2([&writer2_notifier] {
    for (int i = 100; i < 200; ++i) {
      ReadableInfo readable_info;
      readable_info.header_size = i;
      writer2_notifier.Write(readable_info, 1, i, i);
    }
  });

  std::atomic<uint64_t> counter1 = 0;
  std::thread reader1([&reader1_notifier, &counter1] {
    for (int i = 0; i < 200; ++i) {
      ReadableInfo readable_info;
      int index = 0;
      EXPECT_TRUE(reader1_notifier.Read(10, &readable_info, &index));
      counter1.fetch_add(readable_info.header_size);
    }
  });

  std::atomic<uint64_t> counter2 = 0;
  std::thread reader2([&reader2_notifier, &counter2] {
    for (int i = 0; i < 200; ++i) {
      ReadableInfo readable_info;
      int index = 0;
      EXPECT_TRUE(reader2_notifier.Read(10, &readable_info, &index));
      counter2.fetch_add(readable_info.header_size);
    }
  });
  writer1.join();
  writer2.join();
  reader1.join();
  reader2.join();
  EXPECT_EQ(counter1, 19900);
  EXPECT_EQ(counter2, 19900);
}

}  // namespace shm

}  // namespace qcraft
