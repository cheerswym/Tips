#include "onboard/async/thread_pool.h"

#include <string>

#include "gflags/gflags.h"
#include "onboard/utils/thread_util.h"

DEFINE_int32(default_pool_size, 3,
             "The number of threads in the default thread pool");

namespace qcraft {

ThreadPool::ThreadPool(int num_workers,
                       const std::function<void(int index)>& init_thread) {
  QCHECK_GE(num_workers, 0);

  for (int index = 0; index < num_workers; ++index) {
    workers_.emplace_back([this, index, init_thread] {
      std::string str("thread_pool_" + std::to_string(index));
      QSetThreadName(str);
      if (init_thread) {
        init_thread(index);
      }

      while (true) {
        std::function<void()> task;
        {
          absl::MutexLock lock(&mutex_);

          while (!stop_requested_ && tasks_.empty()) {
            cond_var_.Wait(&mutex_);
          }
          if (stop_requested_ && tasks_.empty()) {
            return;
          }
          task = std::move(tasks_.front());
          tasks_.pop();
        }
        task();
      }
    });
  }
}

ThreadPool::~ThreadPool() {
  {
    absl::MutexLock lock(&mutex_);
    stop_requested_ = true;
    cond_var_.SignalAll();
  }
  for (std::thread& worker : workers_) {
    worker.join();
  }
}

// static
ThreadPool* ThreadPool::DefaultPool() {
  static ThreadPool* default_pool = new ThreadPool(FLAGS_default_pool_size);
  return default_pool;
}

// static
ThreadPool* ThreadPool::DisposalPool() {
  static ThreadPool* disposal_pool = new ThreadPool(1);
  return disposal_pool;
}

}  // namespace qcraft
