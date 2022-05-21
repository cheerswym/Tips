#ifndef ONBOARD_UTILS_PERIODIC_RUNNER_H_
#define ONBOARD_UTILS_PERIODIC_RUNNER_H_

#include <algorithm>
#include <atomic>
#include <future>
#include <iostream>
#include <utility>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "onboard/global/car_common.h"
#include "onboard/lite/check.h"
#include "onboard/utils/thread_util.h"

namespace qcraft {

// Start a thread and periodically run a functor with best effort.
// If the functor runs longer than interval , the functor will run immediately
// next.

// Note, this function is not D-Sim friendly. Make sure don't use this for
// anything you would like to run with DSim.

// Onboard or R-sim is safe to use this utility.

class PeriodicRunner {
 public:
  explicit PeriodicRunner(const absl::Duration& interval,
                          const bool ignore_check = false)
      : interval_(interval), stop_requested_(false) {
    if (!ignore_check) {
      QCHECK(IsOnboardMode() || IsRSimMode()) << "Should not be used in D-Sim";
    }
  }

  ~PeriodicRunner() { Stop(); }

  // Run the perodic runner.
  // the Func will run once immeidately.
  template <typename Func>
  void Start(const Func& func) {
    thread_ = std::thread([this, func = func]() {
      VLOG(5) << "start the thread";
      QSetThreadName("PeriodicRunner");
      while (!stop_requested_) {
        last_run_time_ = absl::Now();
        func();  // run immediately.
        auto now = absl::Now();
        if (now < last_run_time_ + interval_) {
          auto sleep_time = last_run_time_ + interval_ - now;
          VLOG(5) << "sleep for " << absl::FormatDuration(sleep_time);
          absl::SleepFor(sleep_time);
        }
      }
    });
  }

  void Stop() {
    if (stop_requested_ != true) {
      stop_requested_ = true;
      thread_.join();
      LOG(INFO) << "PeriodicRunner Stopped";
    }
  }

 private:
  absl::Duration interval_;
  std::atomic<bool> stop_requested_;
  absl::Time last_run_time_;
  std::thread thread_;
};
}  // namespace qcraft

#endif  // ONBOARD_UTILS_PERIODIC_RUNNER_H_
