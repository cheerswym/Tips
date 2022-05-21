#ifndef ONBOARD_ASYNC_FUTURE_H_
#define ONBOARD_ASYNC_FUTURE_H_

#include <future>
#include <utility>

namespace qcraft {

template <typename T>
class Future {
 public:
  Future() = default;
  explicit Future(std::future<T> future) : future_(std::move(future)) {}

  bool IsValid() const { return future_.valid(); }

  // Wait for future to be ready.
  void Wait() const {
    if (future_.valid()) {
      future_.wait();
    }
  }

  // Wait for future to be ready and get the returned value.
  T Get() const { return future_.get(); }

 private:
  std::shared_future<T> future_;
};

}  // namespace qcraft

#endif  // ONBOARD_ASYNC_FUTURE_H_
