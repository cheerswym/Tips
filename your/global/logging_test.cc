#include "onboard/global/logging.h"

#include <unistd.h>

#include <algorithm>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(LoggingTest, TestErrorWithOnboard) {
  std::vector<std::thread> ts;
  constexpr int kCounter = 2;
  std::atomic<int> a = 0;
  for (auto i = 0; i < kCounter; i++) {
    ts.push_back(std::thread([&a] {
      auto i = kCounter * 16;
      while (i--) {
        LOGGING_INTERNAL_STATEFUL_CONDITION(EveryNSec, true, 1.5) {
          a.fetch_add(1);
        }
        usleep(100000);
      }
    }));
  }
  for (auto i = 0; i < kCounter; i++) {
    ts[i].join();
  }

  EXPECT_EQ(a, kCounter + 1);
}
}  // namespace
}  // namespace qcraft
