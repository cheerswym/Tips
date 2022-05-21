#include "onboard/async/parallel_for.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(ParallelForTest, TestReduction) {
  std::atomic<int> count(0);
  const int num_iters = 1000;
  ParallelFor(0, num_iters, [&](int i) { count += i; });
  EXPECT_EQ((num_iters - 1) * num_iters / 2, count);

  count = 0;
  ParallelFor(0, num_iters, ThreadPool::DefaultPool(), {.block_size = 1},
              [&](int i) { count += i; });
  EXPECT_EQ((num_iters - 1) * num_iters / 2, count);
}

TEST(ParallelForTest, TestNullPool) {
  std::atomic<int> count(0);
  const int num_iters = 10;
  ParallelFor(0, num_iters, nullptr, [&](int i) { count += i; });
  EXPECT_EQ((num_iters - 1) * num_iters / 2, count);
}

TEST(ParallelForTest, TestWorkerIndexPool) {
  std::map<int, int> count_per_worker;
  const int num_iters = 10000000;
  const int num_workers = 10;
  ThreadPool thread_pool(num_workers - 1);
  for (int i = 0; i < num_workers; ++i) {
    count_per_worker.emplace(i, 0);
  }
  ParallelFor(0, num_iters, &thread_pool, [&](int worker_index, int i) {
    count_per_worker[worker_index]++;
  });
  int count = 0;
  int max_count = 0;
  for (int i = 0; i < num_workers; ++i) {
    max_count = std::max(max_count, count_per_worker.at(i));
    count += count_per_worker.at(i);
  }
  EXPECT_GT(num_iters, max_count);
  EXPECT_EQ(num_iters, count);
}

TEST(ParallelForTest, NestedParallelFor) {
  std::atomic<int> count(0);
  const int num_iters = 10000;
  ThreadPool thread_pool(10);
  thread_local int nested_level = 0;
  ParallelFor(0, num_iters, &thread_pool, [&](int i) {
    ++nested_level;
    ParallelFor(0, num_iters, &thread_pool,
                [&](parallel_for::WorkerIndex worker_index, int j) {
                  ++nested_level;
                  if (worker_index == 0) {
                    CHECK_EQ(nested_level, 2);
                  } else {
                    CHECK_EQ(nested_level, 1);
                  }
                  count++;
                  --nested_level;
                });
    --nested_level;
  });
  EXPECT_EQ(num_iters * num_iters, count);
}

}  // namespace
}  // namespace qcraft
