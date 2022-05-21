#ifndef ONBOARD_LITE_LITE_TIMER_H_
#define ONBOARD_LITE_LITE_TIMER_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "folly/Executor.h"
#include "google/protobuf/message.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/clock.h"
#include "onboard/proto/execution_issue.pb.h"

namespace qcraft {

// Used to disable timer duration rate check.
extern const absl::Duration kDisableTimerDurationCheck;

// This struct contains all the metadata and timing requirements for a lite
// timer.
struct LiteTimer {
  std::string name;
  std::function<void()> cb;
  // Expected execution period.
  absl::Duration period;
  absl::Time next_execute_time;
  bool one_shot;
};

// TODO(kun):
// 1, Support remove timer with name.
// 2, Set Rate checkers.
// 3, support executor to run timers.

// Thread-safe for adding Timers
class TimerManager {
 public:
  // Adds a timer, die if the timer_name has existed.
  LiteTimer& AddTimerOrDie(const std::string& timer_name,
                           std::function<void()> cb, absl::Duration delay,
                           absl::Duration period, bool one_shot);

  // Returns false, if no timers are scheduled.
  bool HasTimers();

  // Returns next expired time.
  absl::Time GetNextExpiredTime();

  // Executes all timers whose expected time <= clock::now() and remove
  // them if they're one shot.
  void ExecuteTimers();

  // Remove a existed timer, returns true, if has the timer name.
  bool RemoveTimer(const std::string& timer_name);

 private:
  absl::Mutex mu_;
  std::map<std::string, LiteTimer> scheduled_timers_ GUARDED_BY(mu_);
};

class CallbackManager {
 public:
  explicit CallbackManager(folly::Executor* executor)
      : executor_(executor), earlist_schedule_time_(absl::InfiniteFuture()) {}

  void ScheduleCallback(std::function<void()> cb) {
    absl::MutexLock l(&mu_);
    scheduled_callbacks_.emplace_back(std::move(cb));
    const auto now = Clock::Now();
    if (earlist_schedule_time_ > now) {
      earlist_schedule_time_ = now;
    }
  }

  void RunCallbacks() {
    absl::MutexLock l(&mu_);
    for (int i = 0; i < scheduled_callbacks_.size(); ++i) {
      if (executor_ != nullptr) {
        executor_->add(std::move(scheduled_callbacks_[i]));
      } else {
        scheduled_callbacks_[i]();
      }
    }
    scheduled_callbacks_.clear();
    earlist_schedule_time_ = absl::InfiniteFuture();
  }

  absl::Time ScheduledTime() {
    absl::ReaderMutexLock l(&mu_);
    return earlist_schedule_time_;
  }

 private:
  folly::Executor* executor_;
  absl::Mutex mu_;
  std::vector<std::function<void()>> scheduled_callbacks_ GUARDED_BY(mu_);
  absl::Time earlist_schedule_time_ GUARDED_BY(mu_);
};
}  // namespace qcraft

#endif  // ONBOARD_LITE_LITE_TIMER_H_
