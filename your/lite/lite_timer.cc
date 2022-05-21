#include "onboard/lite/lite_timer.h"

#include <algorithm>
#include <set>

#include "glog/logging.h"
#include "onboard/global/car_common.h"
#include "onboard/global/clock.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/proto/execution_issue.pb.h"

namespace qcraft {

constexpr absl::Duration kDisableTimerDurationCheck = absl::ZeroDuration();

LiteTimer& TimerManager::AddTimerOrDie(const std::string& timer_name,
                                       std::function<void()> cb,
                                       const absl::Duration delay,
                                       const absl::Duration period,
                                       const bool one_shot) {
  QCHECK_LT(absl::ZeroDuration(), period);
  absl::MutexLock l(&mu_);
  if (scheduled_timers_.find(timer_name) != scheduled_timers_.end()) {
    QLOG(FATAL) << timer_name << " has existed!";
  }
  LiteTimer timer;
  timer.name = timer_name;
  timer.cb = std::move(cb);
  timer.period = period;
  const auto& next_execute_time = Clock::Now() + period + delay;
  timer.next_execute_time = next_execute_time;
  timer.one_shot = one_shot;
  scheduled_timers_[timer_name] = std::move(timer);
  return scheduled_timers_[timer_name];
}

bool TimerManager::HasTimers() {
  absl::ReaderMutexLock l(&mu_);
  return !scheduled_timers_.empty();
}

absl::Time TimerManager::GetNextExpiredTime() {
  absl::Time expired_time = absl::InfiniteFuture();
  absl::ReaderMutexLock l(&mu_);
  for (const auto& iter : scheduled_timers_) {
    // TODO(kun): Report issue if expired time < clock::Now().
    if (iter.second.next_execute_time < expired_time) {
      expired_time = iter.second.next_execute_time;
    }
  }

  return expired_time;
}

void TimerManager::ExecuteTimers() {
  QCOUNTER_SPAN("TimerManager::ExecuteTimers");

  std::set<std::string> pending_delete_timers;
  std::vector<LiteTimer*> timers_to_run;
  const absl::Time now = Clock::Now();
  {
    absl::MutexLock l(&mu_);
    for (auto& iter : scheduled_timers_) {
      auto& timer = iter.second;
      const auto next_execute_time = timer.next_execute_time;
      if (next_execute_time <= now) {
        if (timer.one_shot) {
          pending_delete_timers.insert(timer.name);
        } else {
          int jump_over = -1;
          do {
            timer.next_execute_time += timer.period;
            jump_over++;
          } while (timer.next_execute_time <= now);
          if (jump_over > 0) {
            QLOG(ERROR) << "Lite timer \"" << iter.first << "\" jump_over "
                        << jump_over << " period: "
                        << jump_over * absl::ToInt64Milliseconds(timer.period)
                        << " ms";
          }
        }

        timers_to_run.push_back(&timer);
      }
    }
  }

  for (auto* timer : timers_to_run) {
    (timer->cb)();
  }

  {
    absl::MutexLock l(&mu_);
    for (const std::string& name : pending_delete_timers) {
      scheduled_timers_.erase(name);
    }
  }
}

bool TimerManager::RemoveTimer(const std::string& timer_name) {
  absl::MutexLock l(&mu_);
  auto iter = scheduled_timers_.find(timer_name);
  if (iter != scheduled_timers_.end()) {
    scheduled_timers_.erase(iter);
    return true;
  }
  return false;
}

}  // namespace qcraft
