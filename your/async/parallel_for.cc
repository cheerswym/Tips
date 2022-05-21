#include "onboard/async/parallel_for.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "onboard/global/trace.h"

namespace qcraft {

namespace {
// This class creates a thread safe barrier which will block until
// Finished() is called.
class BlockUntilFinished {
 public:
  explicit BlockUntilFinished(int num_iters)
      : num_iters_(num_iters), iters_done_(0) {}

  // Signal the blocking thread that all jobs have finished.
  void Finished(int iters_done) {
    absl::MutexLock lock(&mutex_);
    iters_done_ += iters_done;
    cond_var_.Signal();
  }

  // Block until the finished notification arrives.
  void Block() {
    absl::MutexLock lock(&mutex_);
    while (iters_done_ != num_iters_) {
      cond_var_.Wait(&mutex_);
    }
  }

 private:
  absl::Mutex mutex_;
  absl::CondVar cond_var_ GUARDED_BY(mutex_);
  int num_iters_ GUARDED_BY(mutex_);
  int iters_done_ GUARDED_BY(mutex_);
};

// Shared state between the parallel tasks. Each thread will use this
// information to get the next block of work to be performed.
struct SharedState {
  explicit SharedState(int num_iters) : block_until_finished(num_iters) {}

  std::atomic<int> next_index{0};

  // Used to signal when all the work has been completed. Thread safe.
  BlockUntilFinished block_until_finished;
};

}  // namespace

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func) {
  if (end == begin) return;

  // Run this for loop sequentially if thread_pool is null.
  if (thread_pool == nullptr) {
    SCOPED_QTRACE_ARG1("ParallelFor::WorkerThread_serial", "num", end - begin);
    for (int i = begin; i < end; ++i) {
      func(parallel_for::WorkerIndex(0), i);
    }
    return;
  }

  SCOPED_QTRACE_ARG1("ParallelFor", "num", end - begin);

  const int num_iters = end - begin;
  constexpr int kBlockSizeDivider = 4;
  int block_size = options.block_size;
  if (block_size == 0) {
    block_size = std::max<int>(
        1, num_iters / ((thread_pool->NumWorkers() + 1) * kBlockSizeDivider));
  }

  auto shared_state = std::make_shared<SharedState>(num_iters);

  // Dynamically assign tasks to all workers for better load balance.
  const auto grab_tasks = [=, &func](parallel_for::WorkerIndex worker_index) {
    SCOPED_QTRACE("ParallelFor::WorkerThread");
    int iters_done = 0;
    while (true) {
      const int index = shared_state->next_index.fetch_add(
                            block_size, std::memory_order_acq_rel) +
                        begin;
      if (index >= end) break;
      for (int i = index; i < std::min(end, index + block_size);
           ++i, ++iters_done) {
        func(worker_index, i);
      }
    }
    shared_state->block_until_finished.Finished(iters_done);
  };

  const int min_num_workers = (num_iters + block_size - 1) / block_size;
  const int num_workers =
      std::min(min_num_workers - 1, thread_pool->NumWorkers());
  // Schedule futures.
  for (int i = 0; i < num_workers; ++i) {
    thread_pool->Schedule(grab_tasks, parallel_for::WorkerIndex(i + 1));
  }
  // Also run tasks in the master thread.
  grab_tasks(parallel_for::WorkerIndex(0));

  // Wait until all tasks have finished.
  shared_state->block_until_finished.Block();
}

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func) {
  ParallelFor(
      begin, end, thread_pool, {},
      std::forward<std::function<void(parallel_for::WorkerIndex, int)>>(func));
}

void ParallelFor(int begin, int end,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func) {
  ParallelFor(
      begin, end, ThreadPool::DefaultPool(),
      std::forward<std::function<void(parallel_for::WorkerIndex, int)>>(func));
}

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(int)>&& func) {
  ParallelFor(begin, end, thread_pool, options,
              [func = std::move(func)](int worker_index, int i) { func(i); });
}

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(int)>&& func) {
  ParallelFor(begin, end, thread_pool, {},
              [func = std::move(func)](int worker_index, int i) { func(i); });
}

void ParallelFor(int begin, int end, std::function<void(int)>&& func) {
  ParallelFor(begin, end, ThreadPool::DefaultPool(),
              [func = std::move(func)](int worker_index, int i) { func(i); });
}

}  // namespace qcraft
