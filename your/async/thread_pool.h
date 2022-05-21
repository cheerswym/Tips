#ifndef ONBOARD_ASYNC_THREAD_POOL_H_
#define ONBOARD_ASYNC_THREAD_POOL_H_

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/synchronization/mutex.h"
#include "gflags/gflags.h"
#include "onboard/async/future.h"
#include "onboard/global/counter.h"
#include "onboard/lite/check.h"

DECLARE_int32(default_pool_size);

namespace qcraft {

// A thread pool implementation.
// This class is thread safe.
class ThreadPool {
 public:
  // TODO(cong): The init_thread func here is used to install lite context so
  // that we can locate the module that each thread resides in. It won't be
  // needed when we run each module in a standalone process.
  // When num_workers is zero, this thread pool becomes an inline thread pool,
  // and any task schedule to this pool will be implemented in place.
  explicit ThreadPool(int num_workers,
                      const std::function<void(int index)>& init_thread = {});
  ~ThreadPool();

  // Return the default thread pool.
  static ThreadPool* DefaultPool();

  // Return the disposal thread pool to destroy stuff asynchronously.
  static ThreadPool* DisposalPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  int NumWorkers() const { return workers_.size(); }

  template <class Func, class... Args>
  using FutureType = Future<typename std::result_of<Func(Args...)>::type>;

  // Schedule a new task.
  template <class Func, class... Args>
  FutureType<Func, Args...> Schedule(Func&& f, Args&&... args)
      LOCKS_EXCLUDED(mutex_);

 private:
  absl::Mutex mutex_;
  absl::CondVar cond_var_ GUARDED_BY(mutex_);

  std::vector<std::thread> workers_;
  // A thread safe queue protected by condition_ and mutex_.
  std::queue<std::function<void()>> tasks_ GUARDED_BY(mutex_);
  bool stop_requested_ GUARDED_BY(mutex_) = false;
};

template <class Func, class... Args>
ThreadPool::FutureType<Func, Args...> ThreadPool::Schedule(Func&& f,
                                                           Args&&... args) {
  using ReturnType = typename std::result_of<Func(Args...)>::type;
  const auto task = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Func>(f), std::forward<Args>(args)...));
  Future<ReturnType> res(task->get_future());

  // If there is no worker, this is an inline thread pool, and the task will be
  // immediately run on the current thread.
  int64_t tasks_size = -1;
  if (workers_.empty()) {
    (*task)();
  } else {
    absl::MutexLock lock(&mutex_);
    // QCHECK(!stop_requested_) << "The thread pool has been stopped";
    tasks_.emplace([task]() { (*task)(); });
    tasks_size = tasks_.size();
    cond_var_.Signal();
  }
  if (tasks_size >= 0) {
    QCOUNTER("schedulefuture_callback_size", tasks_size);
  }
  return res;
}

}  // namespace qcraft

#endif  // ONBOARD_ASYNC_THREAD_POOL_H_
