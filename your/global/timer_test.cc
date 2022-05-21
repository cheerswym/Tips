#include "onboard/global/timer.h"

#include <chrono>
#include <deque>
#include <iostream>
#include <thread>

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(TimerTest, BoostSteadyTimer) {
  constexpr auto kPeriod = 100;
  constexpr auto kCount = 20;
  SteadyTimer timer = SteadyTimer(kPeriod, [] {
    static auto a = 0;
    static auto last = std::chrono::steady_clock().now();
    static auto total = 0;
    auto now = std::chrono::steady_clock().now();
    auto diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last)
            .count();
    if (diff != 0) {
      a++;
      total += diff;
      if (a == kCount) {
        EXPECT_EQ(total, kCount * kPeriod);
      }
    }

    last = now;
    usleep(kPeriod * 1000);
  });
  timer.Start();
  sleep(2);
}

}  // namespace
}  // namespace qcraft
