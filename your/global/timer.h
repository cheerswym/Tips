#ifndef ONBOARD_GLOBAL_TIMER_H_
#define ONBOARD_GLOBAL_TIMER_H_

#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "boost/asio.hpp"
#include "boost/asio/steady_timer.hpp"

namespace qcraft {

class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string& msg = "")
      : start_(absl::Now()), msg_(msg) {}

  ~ScopedTimer();

 private:
  const absl::Time start_;
  const std::string msg_;
};

struct TimerMark {
  std::string msg;
  absl::Time time;
  int id;

  TimerMark(std::string msg_, absl::Time time_, int id_ = 0)
      : msg(msg_), time(time_), id(id_) {}
};

class ScopedMultiTimer {
 public:
  explicit ScopedMultiTimer(const std::string& msg = "")
      : start_(absl::Now()), msg_(msg) {}

  ~ScopedMultiTimer();

  void Mark(const std::string& mark_msg = "", const int id = 0) {
    marks_.emplace_back(mark_msg, absl::Now(), id);
  }

  void Mark(const std::string& mark_msg, const std::string& id);

  void reset(const absl::Time start = absl::Now()) {
    start_ = start;
    marks_.clear();
  }
  const absl::Time start() const { return start_; }
  const absl::Duration total_duration() const { return absl::Now() - start_; }
  const std::vector<TimerMark>& marks() const { return marks_; }

 private:
  absl::Time start_;
  const std::string msg_;
  std::vector<TimerMark> marks_;
};

class SteadyTimer {
 public:
  explicit SteadyTimer(int period_ms, std::function<void()> cb,
                       bool one_shot = false)
      : timer_(io_), cb_(cb), period_ms_(period_ms), one_shot_(one_shot) {}

  ~SteadyTimer();

  void Start();
  void Stop();

 private:
  void Run();

 private:
  boost::asio::io_service io_;
  boost::asio::steady_timer timer_;
  std::function<void()> cb_;
  int period_ms_;
  bool one_shot_;
  std::thread thread_;
};

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_TIMER_H_
