#ifndef ONBOARD_PERCEPTION_UTILS_TIMER_H_
#define ONBOARD_PERCEPTION_UTILS_TIMER_H_
#include <iostream>
#include <string>
#include <unordered_map>

#include "absl/time/time.h"
#include "onboard/global/clock.h"

namespace qcraft::perception::utils {
class Timer {
 public:
  Timer() {
    duration_ = absl::ZeroDuration();
    start_ = Clock::Now();
    cnt_ = 0;
  }
  void start() { start_ = Clock::Now(); }
  void end() {
    duration_ += Clock::Now() - start_;
    cnt_ += 1;
  }
  void reset() {
    duration_ = absl::ZeroDuration();
    cnt_ = 0;
  }
  double get_avg_time() {
    const double diff_milli = absl::ToDoubleMilliseconds(duration_);
    return diff_milli / cnt_;
  }
  double get_total_time() { return absl::ToDoubleMilliseconds(duration_); }
  double iter_num() { return cnt_; }

 private:
  absl::Time start_;
  absl::Duration duration_;
  int cnt_;
};

class TimeDict {
 public:
  TimeDict() {}
  void start(std::string name) {
    if (data.count(name) == 0) {
      data[name] = Timer();
    }
    data[name].start();
  }
  void end(std::string name) { data[name].end(); }
  void print(std::string name, int norm) {
    const double total_time = data[name].get_total_time();
    std::cout << name << " time: " << total_time / norm << "ms\n";
  }
  void print_all(int norm = 1) {
    std::cout << "iter: " << norm << "\n";
    for (const auto &x : data) {
      print(x.first, norm);
    }
  }

 private:
  std::unordered_map<std::string, Timer> data;
};

}  // namespace qcraft::perception::utils
#endif  // ONBOARD_PERCEPTION_UTILS_TIMER_H_
