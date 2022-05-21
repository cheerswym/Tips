#ifndef ONBOARD_ASYNC_PARALLEL_FOR_H_
#define ONBOARD_ASYNC_PARALLEL_FOR_H_

#include <functional>

#include "onboard/async/thread_pool.h"

namespace qcraft {
namespace parallel_for {

struct Options {
  // Each work will get iterations of this block size. If it is zero, the block
  // size is computed as num_iterations / (num_workers * 4).
  int block_size = 0;
};

// Represents the worker index to not be confused with iteration index.
class WorkerIndex {
 public:
  explicit WorkerIndex(int index) : index_(index) {}
  operator int() const { return index_; }

 private:
  const int index_;
};

}  // namespace parallel_for

// Run func() in parallel in the given thread pool. The function func() may have
// one or two arguments: for two-argument version, the first one is the worker
// index, and the second one is the loop index, which is in the range [begin,
// end); for one-argument version, the worker index is omitted. If the thread
// pool is null, the default thread poll will be used.

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);
void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);
void ParallelFor(int begin, int end,
                 std::function<void(parallel_for::WorkerIndex, int)>&& func);

void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 const parallel_for::Options& options,
                 std::function<void(int)>&& func);
void ParallelFor(int begin, int end, ThreadPool* thread_pool,
                 std::function<void(int)>&& func);
void ParallelFor(int begin, int end, std::function<void(int)>&& func);

}  // namespace qcraft

#endif  // ONBOARD_ASYNC_PARALLEL_FOR_H_
