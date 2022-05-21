#include "onboard/async/async_util.h"

#include <chrono>
#include <thread>

#include "gtest/gtest.h"

namespace qcraft::async {

TEST(AsyncUtilTest, TestSchedule) {
  auto future = ScheduleFuture([] {
    // Sleep for 0.1s.
    using namespace std::chrono_literals;  // NOLINT
    std::this_thread::sleep_for(100ms);
    return 1;
  });
  EXPECT_EQ(1, future.Get());
}

}  // namespace qcraft::async
