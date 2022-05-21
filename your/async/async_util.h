#ifndef ONBOARD_ASYNC_ASYNC_UTIL_H_
#define ONBOARD_ASYNC_ASYNC_UTIL_H_

#include <memory>
#include <utility>

#include "glog/logging.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/trace.h"

namespace qcraft {

// Schedule a task to the given thread pool. If thread_pool is null, run the
// task synchronously on the current thread.
template <typename Func, typename... Args>
auto ScheduleFuture(ThreadPool* thread_pool, Func&& f, Args&&... args) {
  if (thread_pool == nullptr) {
    Future<typename std::result_of<Func(Args...)>::type> future(
        std::async(std::launch::deferred, std::forward<Func>(f),
                   std::forward<Args>(args)...));
    future.Wait();
    return future;
  }
  return thread_pool->Schedule(std::forward<Func>(f),
                               std::forward<Args>(args)...);
}

// Schedule a task to the default thread pool.
template <typename Func, typename... Args>
auto ScheduleFuture(Func&& f, Args&&... args) {
  return ScheduleFuture(ThreadPool::DefaultPool(), std::forward<Func>(f),
                        std::forward<Args>(args)...);
}

// Asynchronously destroy a container.
template <typename ContainerT>
void DestroyContainerAsync(ThreadPool* thread_pool, ContainerT container) {
  ScheduleFuture(thread_pool, [_ = std::move(container)]() mutable {
    SCOPED_QTRACE("DestroyContainerAsync");
    const auto unused = std::move(_);
  });
}

// Same, but use the default thread pool.
template <typename ContainerT>
void DestroyContainerAsync(ContainerT container) {
  DestroyContainerAsync(ThreadPool::DisposalPool(), std::move(container));
}

template <typename T>
void WaitForFuture(const Future<T>& future) {
  SCOPED_QTRACE("WaitForFuture");
  future.Wait();
}

}  // namespace qcraft

#endif  // ONBOARD_ASYNC_ASYNC_UTIL_H_
