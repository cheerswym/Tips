#ifndef ONBOARD_GLOBAL_CLOCK_H_
#define ONBOARD_GLOBAL_CLOCK_H_

#include <memory>

#include "absl/synchronization/mutex.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
namespace qcraft {
class ClockBase {
 public:
  virtual ~ClockBase() {}
  virtual absl::Time Now() = 0;
  virtual void Sleep(absl::Duration duration) = 0;
  virtual bool IsSimClock() = 0;
};

class SystemClock final : public ClockBase {
 public:
  absl::Time Now() override { return absl::Now(); };
  void Sleep(absl::Duration duration) override { absl::SleepFor(duration); }
  bool IsSimClock() override { return false; }
};

class SimulateClock final : public ClockBase {
 public:
  explicit SimulateClock(absl::Time time) : now_time_(time) {}
  SimulateClock() : SimulateClock(absl::UnixEpoch()) {}
  absl::Time Now() override {
    absl::ReaderMutexLock l(&mu_);
    return now_time_;
  };

  void Sleep(absl::Duration duration) override {
    absl::MutexLock l(&mu_);
    now_time_ += duration;
  }

  void SetTime(absl::Time time) {
    absl::MutexLock l(&mu_);
    now_time_ = time;
  }

  bool IsSimClock() override { return true; }

 private:
  absl::Mutex mu_;
  absl::Time now_time_ GUARDED_BY(mu_);
};

// Sync clock to a log start time and with walltime passed rate.
class RealSimClock final : public ClockBase {
 public:
  explicit RealSimClock(absl::Time st_time)
      : log_duration_(absl::Now() - st_time) {}
  RealSimClock() : log_duration_(absl::ZeroDuration()) {}

  absl::Time Now() override {
    absl::ReaderMutexLock l(&mu_);
    return absl::Now() - log_duration_;
  }

  void SetStartTime(absl::Time st_time) {
    absl::MutexLock l(&mu_);
    log_duration_ = absl::Now() - st_time;
  }

  void Sleep(absl::Duration duration) override { absl::SleepFor(duration); }

  bool IsSimClock() override { return false; }

 private:
  absl::Mutex mu_;
  absl::Duration log_duration_ GUARDED_BY(mu_);
};

class RsimClock final : public ClockBase {
 public:
  explicit RsimClock(absl::Duration time_offset) : time_offset_(time_offset) {}
  RsimClock() : time_offset_(absl::ZeroDuration()) {}

  absl::Time Now() override { return absl::Now() - time_offset_; }

  void Sleep(absl::Duration duration) override { absl::SleepFor(duration); }

  bool IsSimClock() override { return false; }

 private:
  absl::Duration time_offset_;
};

class ChronoSystemClock final : public ClockBase {
 public:
  explicit ChronoSystemClock() {}
  absl::Time Now() override {
    return absl::time_internal::FromUnixDuration(
        absl::time_internal::FromChrono(
            std::chrono::system_clock::now().time_since_epoch()));
  };
  void Sleep(absl::Duration duration) override { absl::SleepFor(duration); }
  bool IsSimClock() override { return false; }
};

class ChronoSteadyClock final : public ClockBase {
 public:
  explicit ChronoSteadyClock() {
    duration_ = std::chrono::system_clock::now().time_since_epoch();
    steady_start_ = std::chrono::steady_clock::now();
  }
  absl::Time Now() override {
    return absl::time_internal::FromUnixDuration(
        absl::time_internal::FromChrono(std::chrono::steady_clock::now() -
                                        steady_start_ + duration_));
  };
  void Sleep(absl::Duration duration) override { absl::SleepFor(duration); }
  bool IsSimClock() override { return false; }

 private:
  std::chrono::nanoseconds duration_;
  std::chrono::steady_clock::time_point steady_start_;
};

ClockBase& GetGlobalClock();
void SetGlobalClock(std::shared_ptr<ClockBase> clock);

class Clock {
 public:
  static absl::Time Now() { return GetGlobalClock().Now(); }
  static void Sleep(absl::Duration duration) {
    GetGlobalClock().Sleep(duration);
  }
  static bool IsSimClock() { return GetGlobalClock().IsSimClock(); }
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_CLOCK_H_
