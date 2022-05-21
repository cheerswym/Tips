#include "onboard/lite/lite_client.h"

#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/lite_shm_message.h"
#include "onboard/lite/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/testdata/lite_msg_test.pb.h"
#include "onboard/lite/transport.h"
#include "onboard/lite/transport/message/inner_message_hub.h"

namespace qcraft {
namespace {

void ClientDispatchLoop(LiteClientBase* client,
                        std::atomic<bool>* stop_running) {
  while (!stop_running->load()) {
    client->SleepUntil(absl::Milliseconds(30));
  }
}

void PublishFakeCounterThread(LiteClientBase* client,
                              const std::string& channel,
                              absl::Duration sleep_duration,
                              int32_t loop_range) {
  for (int i = 1; i <= loop_range; i++) {
    FakeCounter counter;
    counter.set_count(i);
    absl::SleepFor(sleep_duration);
    QLOG_IF_NOT_OK(WARNING, client->Publish(counter, channel));
  }
}

class LiteClientTest : public testing::Test {
 public:
  void FakeSumCallback(std::shared_ptr<const FakeSum> msg) {
    ASSERT_EQ(msg->header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_2));
    EXPECT_EQ(expected_seq_num_2_, msg->header().seq_number());
    expected_seq_num_2_++;
    fake_sum_ += msg->sum();
  }

 protected:
  void SetUp() override {
    GlobalInnerMessageHub()->Reset();
    registered_cnt_[std::make_pair("shm_simple_struct", "")] = 1;
    ShmFactory::GetShmManager()->Reset();
  }

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

  uint64_t expected_seq_num_2_;
  double fake_sum_;
  std::map<std::pair<std::string, std::string>, int32> registered_cnt_;
};

TEST_F(LiteClientTest, SimplePubSubDisabledPublish) {
  expected_seq_num_2_ = 1;
  fake_sum_ = 0;
  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  FakeSum fake_sum;
  fake_sum.set_sum(0.1);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));
  client_1.SleepUntil(absl::Milliseconds(10));

  client_1.Subscribe(&LiteClientTest::FakeSumCallback,
                     static_cast<LiteClientTest*>(this));
  fake_sum.set_sum(0.2);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));

  client_1.SleepUntil(absl::Milliseconds(10));

  EXPECT_EQ(expected_seq_num_2_, 2);
  EXPECT_EQ(fake_sum_, 0.2);
}

TEST_F(LiteClientTest, CheckHeader) {
  LiteClient client(TEST_LITE_MODULE_2,
                    std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  FakeSum fake_sum;
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(i, client.Publish(fake_sum)->seq_number());
  }
}

TEST_F(LiteClientTest, SimplePubSub) {
  expected_seq_num_2_ = 1;
  fake_sum_ = 0;
  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  FakeSum fake_sum;
  fake_sum.set_sum(0.1);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));
  client_1.SleepUntil(absl::Milliseconds(10));
  // No subscribe yet.
  EXPECT_EQ(expected_seq_num_2_, 1);
  EXPECT_EQ(fake_sum_, 0.0);

  client_1.Subscribe(&LiteClientTest::FakeSumCallback,
                     static_cast<LiteClientTest*>(this));
  fake_sum.set_sum(0.2);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));

  // Zero duration, will cause negative duration in practice, so no message get
  // delivered.
  client_1.SleepUntil(absl::ZeroDuration());
  EXPECT_EQ(expected_seq_num_2_, 1);
  EXPECT_EQ(fake_sum_, 0.0);

  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(expected_seq_num_2_, 2);
  EXPECT_EQ(fake_sum_, 0.2);

  fake_sum.set_sum(0.3);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));
  fake_sum.set_sum(0.4);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));

  // client_1.SleepUntil(absl::ZeroDuration());
  client_1.SleepUntil(absl::Milliseconds(10));

  EXPECT_EQ(expected_seq_num_2_, 4);
  EXPECT_EQ(fake_sum_, 0.9);
}

TEST_F(LiteClientTest, OneSubMultiPub) {
  uint64_t expected_seq_num_2 = 0;
  double sum = 0;
  std::function<void(std::shared_ptr<const FakeSum>)> callback =
      [&expected_seq_num_2, &sum](std::shared_ptr<const FakeSum> msg) -> void {
    ASSERT_EQ(msg->header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_2));
    EXPECT_EQ(expected_seq_num_2, msg->header().seq_number());
    expected_seq_num_2++;
    sum += msg->sum();
  };

  uint64_t expected_seq_num_3 = 0;
  std::function<void(std::shared_ptr<const FakeCounter>)> callback_1 =
      [&expected_seq_num_3](std::shared_ptr<const FakeCounter> msg) -> void {
    ASSERT_EQ(msg->header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_3));
    EXPECT_EQ(expected_seq_num_3, msg->header().seq_number());
    expected_seq_num_3++;
    EXPECT_EQ(expected_seq_num_3, msg->count());
  };

  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_1.Subscribe(callback);
  client_1.Subscribe(callback_1);
  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  LiteClient client_3(TEST_LITE_MODULE_3,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  FakeSum fake_sum;
  fake_sum.set_sum(0.1);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));
  fake_sum.set_sum(0.2);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));
  fake_sum.set_sum(0.3);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));
  fake_sum.set_sum(0.4);
  QLOG_IF_NOT_OK(WARNING, client_2.Publish(fake_sum));

  FakeCounter counter;
  counter.set_count(1);
  QLOG_IF_NOT_OK(WARNING, client_3.Publish(counter));

  client_1.SleepUntil(absl::Milliseconds(100));

  counter.set_count(2);
  QLOG_IF_NOT_OK(WARNING, client_3.Publish(counter));
  counter.set_count(3);
  QLOG_IF_NOT_OK(WARNING, client_3.Publish(counter));
  client_1.SleepUntil(absl::Milliseconds(100));

  counter.set_count(4);
  QLOG_IF_NOT_OK(WARNING, client_3.Publish(counter));
  client_1.SleepUntil(absl::Milliseconds(100));

  counter.set_count(5);
  QLOG_IF_NOT_OK(WARNING, client_3.Publish(counter));
  client_1.SleepUntil(absl::Milliseconds(100));

  EXPECT_EQ(expected_seq_num_2, 4);
  EXPECT_EQ(sum, 1.0);
  EXPECT_EQ(expected_seq_num_3, 5);
}

TEST_F(LiteClientTest, MultiThreadsPubAndSub) {
  const std::string kCounter1 = "Counter1";
  const std::string kCounter2 = "Counter2";

  std::atomic<int32_t> counter_1 = 0;
  std::atomic<int32_t> counter_2 = 0;
  std::function<void(std::shared_ptr<const FakeCounter>)> callback_1 =
      [&counter_1](std::shared_ptr<const FakeCounter> msg) -> void {
    counter_1.fetch_add(msg->count());
  };

  std::function<void(std::shared_ptr<const FakeCounter>)> callback_2 =
      [&counter_2](std::shared_ptr<const FakeCounter> msg) -> void {
    counter_2.fetch_add(msg->count());
  };

  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_1.Subscribe(callback_1, kCounter1);
  client_1.Subscribe(callback_2, kCounter2);

  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_2.Subscribe(callback_1, kCounter1);
  client_2.Subscribe(callback_2, kCounter2);

  LiteClient client_3(TEST_LITE_MODULE_3,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_3.Subscribe(callback_1, kCounter1);
  client_3.Subscribe(callback_2, kCounter2);

  std::atomic<bool> stop_running = false;
  std::thread thread_1(ClientDispatchLoop, &client_1, &stop_running);
  std::thread thread_2(ClientDispatchLoop, &client_2, &stop_running);
  std::thread thread_3(ClientDispatchLoop, &client_3, &stop_running);

  LiteClient client_4(TEST_LITE_MODULE_4,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  LiteClient client_5(TEST_LITE_MODULE_5,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  std::vector<std::thread> publish_threads;
  publish_threads.emplace_back(std::thread(PublishFakeCounterThread, &client_4,
                                           kCounter1, absl::Milliseconds(20),
                                           4));
  publish_threads.emplace_back(std::thread(PublishFakeCounterThread, &client_4,
                                           kCounter2, absl::Milliseconds(10),
                                           4));
  publish_threads.emplace_back(std::thread(PublishFakeCounterThread, &client_5,
                                           kCounter1, absl::Milliseconds(10),
                                           4));
  publish_threads.emplace_back(std::thread(PublishFakeCounterThread, &client_5,
                                           kCounter2, absl::Milliseconds(20),
                                           4));
  publish_threads.emplace_back(std::thread(PublishFakeCounterThread, &client_5,
                                           kCounter2, absl::Milliseconds(30),
                                           4));

  for (int i = 0; i < publish_threads.size(); ++i) {
    publish_threads[i].join();
  }
  // Stop sub clients.
  absl::SleepFor(absl::Seconds(1));
  stop_running.store(true);
  thread_1.join();
  thread_2.join();
  thread_3.join();

  EXPECT_EQ(counter_1.load(), 60);
  EXPECT_EQ(counter_2.load(), 90);
}

TEST_F(LiteClientTest, TimerAndSchedule) {
  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  std::atomic<int32_t> counter_1 = 0;
  std::atomic<int32_t> counter_2 = 0;
  std::atomic<int32_t> sum_counter = 0;
  std::atomic<bool> reach_six = false;
  std::atomic<bool> reach_ten = false;
  std::function<void()> timer_1 = [&counter_1, client = &client_1]() -> void {
    counter_1 += 1;
    FakeCounter counter;
    counter.set_count(counter_1.load());
    QLOG_IF_NOT_OK(WARNING, client->Publish(counter));
  };

  std::function<void(std::shared_ptr<const FakeCounter>)> sub_callback =
      [&counter_2, &sum_counter, &reach_six, &reach_ten,
       client = &client_2](std::shared_ptr<const FakeCounter> msg) -> void {
    sum_counter.fetch_add(msg->count());
    counter_2 += 1;
    if (sum_counter == 6 && counter_2 == 3) {
      // When receive 3rd message.
      client->AddTimerOrDie(
          "Set reach_six",
          [&reach_six, &reach_ten]() {
            // Even this timer is called first, but will be
            // executed after reach ten.
            ASSERT_TRUE(reach_ten);
            reach_six.store(true);
          },
          absl::Milliseconds(150), absl::Milliseconds(150));
    } else if (sum_counter.load() == 10 && counter_2 == 4) {
      // When receive 4rd message.
      client->Schedule([&reach_six, &reach_ten]() {
        // reach_six should be set after this scheduled callback.
        ASSERT_FALSE(reach_six);
        reach_ten.store(true);
      });
    }
  };
  client_1.AddTimerOrDie("repeated_publish_counter", timer_1,
                         absl::Milliseconds(100), absl::Milliseconds(100),
                         false /*=one_shot*/);
  client_2.Subscribe(sub_callback);
  std::atomic<bool> stop_running = false;
  std::thread thread_1(ClientDispatchLoop, &client_1, &stop_running);
  std::thread thread_2(ClientDispatchLoop, &client_2, &stop_running);
  // Stop loop.
  absl::SleepFor(absl::Seconds(1));
  stop_running.store(true);
  thread_1.join();
  thread_2.join();
  EXPECT_GE(counter_1, counter_2);
  // The timer gets called at most 10 times, because of 1s / 0.1s.
  EXPECT_LE(counter_1, 10);
  EXPECT_TRUE(reach_six);
  EXPECT_TRUE(reach_ten);
}

TEST_F(LiteClientTest, SimplePubSubShmMsg) {
  uint64_t expected_seq_num_2 = 0;
  std::function<void(std::shared_ptr<ShmMessage>)> callback =
      [&expected_seq_num_2](std::shared_ptr<ShmMessage> msg) -> void {
    const auto& metadata = msg->shm_msg_metadata();
    ASSERT_EQ(metadata.header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_2));
    EXPECT_EQ(expected_seq_num_2, metadata.header().seq_number());
    const SimpleStruct& v = msg->value<SimpleStruct>();
    if (expected_seq_num_2 == 0) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i));
        EXPECT_EQ(v.chars[i], 'a' + i);
      }
    } else if (expected_seq_num_2 == 1) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i + 1));
        EXPECT_EQ(v.chars[i], 'b' + i);
      }
    }
    expected_seq_num_2++;
  };
  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_1.SubscribeShmMsg(callback, "shm_simple_struct");

  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "shm_simple_struct");
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  // Fill SimpleStruct
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  Print(shm_stru1->value<SimpleStruct>());
  QLOG_IF_NOT_OK(WARNING, client_2.PublishShmMsg(shm_stru1.get()));
  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(expected_seq_num_2, 1);

  // Change the content
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i + 1);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'b' + i;
  }

  Print(shm_stru1->value<SimpleStruct>());
  QLOG_IF_NOT_OK(WARNING, client_2.PublishShmMsg(shm_stru1.get()));
  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(expected_seq_num_2, 2);
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  shm_stru1.reset();
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
}

TEST_F(LiteClientTest, PubSubShmMsgDisabled) {
  uint64_t expected_seq_num_2 = 0;
  std::function<void(std::shared_ptr<ShmMessage>)> callback =
      [&](std::shared_ptr<ShmMessage> msg) -> void { expected_seq_num_2++; };
  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_1.SubscribeShmMsg(callback, "shm_simple_struct");
  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "shm_simple_struct");
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  // Fill SimpleStruct
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  Print(shm_stru1->value<SimpleStruct>());

  QLOG_IF_NOT_OK(WARNING, client_2.PublishShmMsg(shm_stru1.get()));
  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(expected_seq_num_2, 1);
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  shm_stru1.reset();
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
}

// TODO(kun): Add more test for share memory test.

TEST_F(LiteClientTest, PubSubShmMsgWithOnlyOneReader) {
  uint64_t expected_seq_num_1 = 0;
  uint64_t expected_seq_num_2 = 0;
  std::function<void(std::shared_ptr<ShmMessage>)> callback_1 =
      [expected_seq_num =
           &expected_seq_num_1](std::shared_ptr<ShmMessage> msg) -> void {
    const auto& metadata = msg->shm_msg_metadata();
    ASSERT_EQ(metadata.header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_3));
    EXPECT_EQ(*expected_seq_num, metadata.header().seq_number());
    LOG(ERROR) << "header: " << metadata.header().DebugString();
    const SimpleStruct& v = msg->value<SimpleStruct>();
    if (*expected_seq_num == 0) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i));
        EXPECT_EQ(v.chars[i], 'a' + i);
      }
    } else if (*expected_seq_num == 1) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i + 1));
        EXPECT_EQ(v.chars[i], 'b' + i);
      }
    }
    (*expected_seq_num)++;
  };

  std::function<void(std::shared_ptr<ShmMessage>)> callback_2 =
      [expected_seq_num =
           &expected_seq_num_2](std::shared_ptr<ShmMessage> msg) -> void {
    (*expected_seq_num)++;
    EXPECT_EQ(msg, nullptr);
  };

  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_1.SubscribeShmMsg(std::move(callback_1), "shm_simple_struct");

  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_2.SubscribeShmMsg(std::move(callback_2), "shm_simple_struct");

  LiteClient client_3(TEST_LITE_MODULE_3,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  ShmFactory::GetShmManager()->TEST_SetRegisteredShmMsgReaderCount(
      registered_cnt_);

  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "shm_simple_struct");
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  // Fill SimpleStruct
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  Print(shm_stru1->value<SimpleStruct>());
  QLOG_IF_NOT_OK(WARNING, client_3.PublishShmMsg(shm_stru1.get()));
  shm_stru1.reset();
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  client_2.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(expected_seq_num_1, 1);
  EXPECT_EQ(expected_seq_num_2, 0);
}

TEST_F(LiteClientTest, MultiplePubSubShmMsgs) {
  uint64_t expected_seq_num_1 = 0;
  uint64_t expected_seq_num_2 = 0;
  std::function<void(std::shared_ptr<ShmMessage>)> callback_1 =
      [expected_seq_num =
           &expected_seq_num_1](std::shared_ptr<ShmMessage> msg) -> void {
    const auto& metadata = msg->shm_msg_metadata();
    ASSERT_EQ(metadata.header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_3));
    EXPECT_EQ(*expected_seq_num, metadata.header().seq_number());
    LOG(ERROR) << "header: " << metadata.header().DebugString();
    const SimpleStruct& v = msg->value<SimpleStruct>();
    if (*expected_seq_num == 0) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i));
        EXPECT_EQ(v.chars[i], 'a' + i);
      }
    } else if (*expected_seq_num == 1) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i + 1));
        EXPECT_EQ(v.chars[i], 'b' + i);
      }
    }
    (*expected_seq_num)++;
  };

  std::function<void(std::shared_ptr<ShmMessage>)> callback_2 =
      [expected_seq_num =
           &expected_seq_num_2](std::shared_ptr<ShmMessage> msg) -> void {
    const auto& metadata = msg->shm_msg_metadata();
    ASSERT_EQ(metadata.header().module_id(),
              static_cast<int32_t>(TEST_LITE_MODULE_3));
    EXPECT_EQ(*expected_seq_num, metadata.header().seq_number());
    LOG(ERROR) << "header: " << metadata.header().DebugString();
    const SimpleStruct& v = msg->value<SimpleStruct>();
    if (*expected_seq_num == 0) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i));
        EXPECT_EQ(v.chars[i], 'a' + i);
      }
    } else if (*expected_seq_num == 1) {
      EXPECT_EQ(v.size, 10);
      for (int i = 0; i < 10; i++) {
        EXPECT_EQ(v.numbers[i], static_cast<double>(i + 1));
        EXPECT_EQ(v.chars[i], 'b' + i);
      }
    }
    (*expected_seq_num)++;
  };

  LiteClient client_1(TEST_LITE_MODULE_1,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_1.SubscribeShmMsg(std::move(callback_1), "shm_simple_struct");

  LiteClient client_2(TEST_LITE_MODULE_2,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));
  client_2.SubscribeShmMsg(std::move(callback_2), "shm_simple_struct");

  LiteClient client_3(TEST_LITE_MODULE_3,
                      std::make_unique<LiteTransport>(GlobalInnerMessageHub()));

  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);
  registered_cnt_[std::make_pair("shm_simple_struct", "")] = 2;
  ShmFactory::GetShmManager()->TEST_SetRegisteredShmMsgReaderCount(
      registered_cnt_);
  std::unique_ptr<ShmMessage> shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "shm_simple_struct");
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  // Fill SimpleStruct
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'a' + i;
  }
  Print(shm_stru1->value<SimpleStruct>());
  QLOG_IF_NOT_OK(WARNING, client_3.PublishShmMsg(shm_stru1.get()));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  client_2.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  EXPECT_EQ(expected_seq_num_1, 1);
  EXPECT_EQ(expected_seq_num_2, 1);

  shm_stru1.reset();
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 0);

  shm_stru1 =
      ShmMessage::CreateToWrite<SimpleStruct>(SHM_UNKNOWN, "shm_simple_struct");
  ASSERT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);

  // Change the content
  shm_stru1->mutable_value<SimpleStruct>()->size = 10;
  for (int i = 0; i < 10; i++) {
    shm_stru1->mutable_value<SimpleStruct>()->numbers[i] =
        static_cast<double>(i + 1);
    shm_stru1->mutable_value<SimpleStruct>()->chars[i] = 'b' + i;
  }
  QLOG_IF_NOT_OK(WARNING, client_3.PublishShmMsg(shm_stru1.get()));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  client_1.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  client_2.SleepUntil(absl::Milliseconds(10));
  EXPECT_EQ(ShmFactory::GetShmManager()->NumOfAvaibleShmBlocks(), 1);
  EXPECT_EQ(expected_seq_num_1, 2);
  EXPECT_EQ(expected_seq_num_2, 2);
}

}  // namespace
}  // namespace qcraft
