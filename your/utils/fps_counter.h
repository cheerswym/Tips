#ifndef ONBOARD_UTILS_FPS_COUNTER_H_
#define ONBOARD_UTILS_FPS_COUNTER_H_

#include <string>

#include "absl/strings/str_format.h"
#include "glog/logging.h"

namespace qcraft {

class FpsCounter {
 public:
  explicit FpsCounter(const std::string tag = "") : tag_(tag) {}
  void Tic(double timestamp) {
    if (timestamp - prev_time_ > 0.99) {
      if (n_ > 0) {
        last_fps_ = n_ - 1 / (timestamp - prev_time_);
      }
      prev_time_ = timestamp;
      n_ = 1;
    }
    ++n_;
  }

  double last_fps() { return last_fps_; }

 private:
  std::string tag_;
  double last_fps_;
  double prev_time_ = -1.;
  int n_ = 0;
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_FPS_COUNTER_H_
