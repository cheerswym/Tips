#include "onboard/global/counter.h"

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(CounterTest, Test_QCOUNTER_simple) {
  for (int i = 0; i < 100; i++) {
    QCOUNTER("bar", i);
  }
  auto output = Counter::Instance()->GetCounterOutput();
  auto item0 = output.item(0);
  EXPECT_EQ(item0.name(), "bar");
  EXPECT_EQ(item0.max(), 99);
  EXPECT_EQ(item0.min(), 0);
  EXPECT_EQ(item0.count(), 100);
  EXPECT_EQ(item0.sum(), 4950);
}

TEST(CounterTest, Test_QCOUNTER_Multithread) {
  std::mutex m;
  std::condition_variable cv;
  int count = 0;
  auto t1 = std::thread([&]() {
    std::unique_lock<std::mutex> lk(m);
    std::string a;
    for (int i = 1; i <= 5; i++) {
      QCOUNTER("foo", i);  // foo: 1,2,3,4,5
    }
    for (int i = 6; i <= 10; i++) {
      QCOUNTER("bar", i);  // bar: 6,7,8,9,10
    }
    QCOUNTER("bar", 100);
    count++;
    lk.unlock();
    cv.notify_all();
    std::unique_lock<std::mutex> lk_keep(m);
    cv.wait(lk_keep, [&] { return count == 3; });
    lk_keep.unlock();
  });

  auto t2 = std::thread([&]() {
    std::unique_lock<std::mutex> lk(m);
    for (int i = 1; i <= 2; i++) {
      QCOUNTER("foo", i * 10);  // foo 10,20
    }
    for (int i = 3; i <= 4; i++) {
      QCOUNTER("bar", i * 10);  // bar 30,40
    }
    count++;
    lk.unlock();
    cv.notify_all();
    std::unique_lock<std::mutex> lk_keep(m);
    cv.wait(lk_keep, [&] { return count == 3; });
    lk_keep.unlock();
  });

  auto t3 = std::thread([&]() {
    std::unique_lock<std::mutex> lk(m);
    cv.wait(lk, [&] { return count == 2; });
    auto output = Counter::Instance()->GetCounterOutput();
    EXPECT_EQ(output.item_size(), 2);

    auto item0 = output.item(0);
    EXPECT_EQ(item0.name(), "bar");
    EXPECT_EQ(item0.max(), 100);
    EXPECT_EQ(item0.min(), 6);
    EXPECT_EQ(item0.count(), 8);
    EXPECT_EQ(item0.sum(), 210);

    auto item1 = output.item(1);
    EXPECT_EQ(item1.name(), "foo");
    EXPECT_EQ(item1.max(), 20);
    EXPECT_EQ(item1.min(), 1);
    EXPECT_EQ(item1.count(), 7);
    EXPECT_EQ(item1.sum(), 45);
    count++;
    lk.unlock();
    cv.notify_all();
  });
  t1.join();
  t2.join();
  t3.join();
}

TEST(QCOUNTER_SPAN, Test_QCOUNTER_SPAN_simple) {
  constexpr auto millseconds = 1000;
  {
    QCOUNTER_SPAN("QCOUNTER_SPAN");
    usleep(millseconds * 1000);
  }

  auto output = Counter::Instance()->GetCounterOutput();
  EXPECT_EQ(output.item_size(), 1);
  auto item0 = output.item(0);
  EXPECT_EQ(item0.name(), "QCOUNTER_SPAN");
}

}  // namespace
}  // namespace qcraft
