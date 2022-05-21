#include "onboard/utils/lite_msg_stats.h"

#include "gtest/gtest.h"

namespace qcraft {
TEST(LiteMsgStatsTest, EmptyTest) {
  LiteMsgStats stat("lidar", "Spin");
  EXPECT_EQ(stat.TopicName(), "lidar");
  EXPECT_EQ(stat.MsgType(), "Spin");
  EXPECT_DOUBLE_EQ(stat.AvgMsgSize(), 0.0);
  EXPECT_DOUBLE_EQ(stat.Hz(), 0.0);
}

TEST(LiteMsgStatsTest, GotMsgTest) {
  LiteMsgStats stat("lidar", "Spin");
  stat.GotNewMessage(1000, 2);
  stat.GotNewMessage(2000, 4);
  EXPECT_DOUBLE_EQ(stat.AvgMsgSize(), 3.0);
  EXPECT_DOUBLE_EQ(stat.Hz(), 1000);
  stat.Clear();
  stat.GotNewMessage(10, 5);
  stat.GotNewMessage(15, 5);
  EXPECT_DOUBLE_EQ(stat.Hz(), 1.0 * LiteMsgStats::kFreqMax);
  stat.Clear();
  int i = 1;
  const int& kWindowSize = LiteMsgStats::kWindowSize;
  for (; i <= kWindowSize; ++i) {
    stat.GotNewMessage(i * 20, 1024);
  }
  EXPECT_DOUBLE_EQ(stat.Hz(), 50000.0);
  for (i = 1; i <= 2 * kWindowSize; ++i) {
    stat.GotNewMessage(kWindowSize * 20 + i * 100, 1024);
  }
  EXPECT_DOUBLE_EQ(stat.Hz(), 10000.0);
}

}  // namespace qcraft
