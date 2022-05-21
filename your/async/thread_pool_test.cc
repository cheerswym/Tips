#include "onboard/async/thread_pool.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(ThreadPoolTest, ThreadPoolTest) {
  std::atomic<int> count(0);
  const auto increase_count = [&count]() { count++; };
  ThreadPool thread_pool(4);
  const int num_iters = 100;
  std::vector<Future<void>> futures;
  for (int i = 0; i < num_iters; ++i) {
    futures.push_back(thread_pool.Schedule(increase_count));
  }
  for (const auto& future : futures) {
    future.Wait();
  }
  EXPECT_EQ(num_iters, count);
}

TEST(ThreadPoolTest, TestInlineThreadPool) {
  ThreadPool thread_pool(0);
  absl::Mutex mutex;
  std::vector<int> nums;
  for (int i = 0; i < 100; ++i) {
    thread_pool.Schedule([&] {
      absl::MutexLock lock(&mutex);
      nums.push_back(i);
    });
  }
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(i, nums[i]);
  }
}

}  // namespace
}  // namespace qcraft
