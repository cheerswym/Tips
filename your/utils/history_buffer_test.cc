#include "onboard/utils/history_buffer.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(HistoryBufferTest, DirectRandomAccess) {
  HistoryBuffer<int> buffer;
  buffer.push_back(1.0, 10);
  buffer.push_back(2.0, 20);
  buffer.push_back(3.0, 30);
  buffer.push_front(0.0, 0);
  buffer.push_back(4.0, 40);
  EXPECT_EQ(buffer.time(0), 0.0);
  EXPECT_EQ(buffer.time(4), 4.0);
  EXPECT_EQ(buffer.value(0), 0);
  EXPECT_EQ(buffer.value(4), 40);
  buffer.value(2) = 100;
  EXPECT_EQ(buffer.value(2), 100);
}

TEST(HistoryBufferTest, RangeBasedIteration) {
  HistoryBuffer<int> buffer;
  buffer.push_back(0.0, 0);
  buffer.push_back(1.0, 10);
  buffer.push_back(2.0, 20);
  buffer.push_back(3.0, 30);
  buffer.push_back(4.0, 40);
  int c = 0;
  for (const auto &kv : buffer) {
    EXPECT_EQ(c * 10, kv.second);
    ++c;
  }

  c = 0;
  for (const double t : buffer.time_range()) {
    EXPECT_EQ(c * 1.0, t);
    ++c;
  }

  c = 0;
  for (const int v : buffer.value_range()) {
    EXPECT_EQ(c * 10, v);
    ++c;
  }
}

TEST(HistoryBufferTest, Search) {
  HistoryBuffer<int> buffer;
  buffer.push_back(1.0, 10);
  buffer.push_back(2.0, 20);
  buffer.push_back(2.0, 30);
  buffer.push_back(3.0, 40);

  EXPECT_EQ(buffer.FindTimeAtLeast(0.5), 1.0);
  EXPECT_EQ(buffer.FindTimeAtLeast(1.0), 1.0);
  EXPECT_EQ(buffer.FindTimeAtLeast(1.5), 2.0);
  EXPECT_EQ(buffer.FindTimeAtLeast(2.0), 2.0);
  EXPECT_EQ(buffer.FindTimeAtLeast(2.5), 3.0);
  EXPECT_EQ(buffer.FindTimeAtLeast(3.0), 3.0);
  EXPECT_EQ(buffer.FindTimeAtLeast(3.5), 0.0);  // Not found.
  EXPECT_EQ(buffer.FindTimeAtMost(0.5), 0.0);   // Not found.
  EXPECT_EQ(buffer.FindTimeAtMost(1.0), 1.0);
  EXPECT_EQ(buffer.FindTimeAtMost(1.5), 1.0);
  EXPECT_EQ(buffer.FindTimeAtMost(2.0), 2.0);
  EXPECT_EQ(buffer.FindTimeAtMost(2.5), 2.0);
  EXPECT_EQ(buffer.FindTimeAtMost(3.0), 3.0);
  EXPECT_EQ(buffer.FindTimeAtMost(3.5), 3.0);
  EXPECT_EQ(buffer.FindTimeClosest(1.6), 2.0);
  EXPECT_EQ(buffer.FindTimeClosest(0.6), 1.0);
  EXPECT_EQ(*buffer.GetValueWithClosestTime(0.6), 10);
  EXPECT_EQ(*buffer.GetValueWithClosestTime(2.6), 40);

  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(0.5), 0);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(1.0), 0);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(1.5), 1);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(2.0), 1);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(2.5), 3);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(3.0), 3);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(3.5), 4);  // Not found.
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(0.5), -1);  // Not found.
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(1.0), 0);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(1.5), 0);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(2.0), 2);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(2.5), 2);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(3.0), 3);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(3.5), 3);
}

TEST(HistoryBufferTest, ClearStale) {
  HistoryBuffer<int> buffer;
  buffer.push_back(1.0, 10);
  buffer.push_back(2.0, 20);
  buffer.push_back(2.0, 30);
  buffer.push_back(3.0, 40);
  buffer.ClearOlderThan(1.5);
  EXPECT_EQ(buffer.size(), 3);
  EXPECT_EQ(buffer.front_time(), 2.0);
  buffer.ClearOlderThan(0.5);
  EXPECT_EQ(buffer.size(), 1);
  EXPECT_EQ(buffer.front_time(), 3.0);
}

TEST(HistoryBufferTest, Insert) {
  HistoryBuffer<int> buffer;
  buffer.push_back(1.0, 10);
  buffer.push_back(2.0, 20);
  buffer.push_back(2.0, 30);
  buffer.push_back(3.0, 40);
  buffer.Insert(1.5, 15);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(1.5), 1);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(1.5), 1);
  buffer.Insert(2.5, 25);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(2.5), 4);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(2.5), 4);
}

TEST(HistoryBufferTest, AbslTime) {
  HistoryBufferAbslTime<int> buffer;
  buffer.push_back(absl::FromUnixMillis(1000), 10);
  buffer.push_back(absl::FromUnixMillis(2000), 20);
  buffer.Insert(absl::FromUnixMillis(1500), 15);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(absl::FromUnixMillis(1500)), 1);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(absl::FromUnixMillis(1500)), 1);
  buffer.ClearOlderThan(absl::Milliseconds(800));
  EXPECT_EQ(buffer.size(), 2);
  EXPECT_EQ(buffer.front_time(), absl::FromUnixMillis(1500));
}

TEST(HistoryBufferTest, StdTime) {
  using Clock = std::chrono::system_clock;
  using Time = std::chrono::time_point<Clock>;
  HistoryBufferStdTime<int, Clock> buffer;
  buffer.push_back(Time(std::chrono::seconds(10)), 10);
  buffer.push_back(Time(std::chrono::seconds(20)), 20);
  buffer.Insert(Time(std::chrono::seconds(15)), 15);
  EXPECT_EQ(buffer.GetIndexWithTimeAtLeast(Time(std::chrono::seconds(15))), 1);
  EXPECT_EQ(buffer.GetIndexWithTimeAtMost(Time(std::chrono::seconds(15))), 1);
  buffer.ClearOlderThan(std::chrono::seconds(8));
  EXPECT_EQ(buffer.size(), 2);
  EXPECT_EQ(buffer.front_time(), Time(std::chrono::seconds(15)));
}

}  // namespace
}  // namespace qcraft
