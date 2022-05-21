#include "onboard/global/timer.h"

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/utils/thread_util.h"

DEFINE_bool(log_trace, false, "Show tracing results in logs");

namespace qcraft {

ScopedTimer::~ScopedTimer() {
  if (FLAGS_log_trace) {
    LOG(INFO) << (msg_.empty() ? "ScopedTimer" : msg_) << ": "
              << absl::Now() - start_;
  }
}

ScopedMultiTimer::~ScopedMultiTimer() {
  if (FLAGS_log_trace) {
    LOG(INFO) << (msg_.empty() ? "ScopedMultiTimer" : msg_) << ":";
    absl::Time prev_time = start_;
    for (const auto& kv : marks_) {
      LOG(INFO) << "  [" << kv.msg << "]: " << (kv.time - prev_time);
      prev_time = kv.time;
    }
    const absl::Time now = absl::Now();
    LOG(INFO) << "  [End]: " << now - prev_time;
    LOG(INFO) << "  [Total]: " << now - start_;
  }
}

void ScopedMultiTimer::Mark(const std::string& mark_msg,
                            const std::string& id) {
  int digit_begin = -1;
  int digit_end = id.size();
  for (int i = 0; i < id.size(); ++i) {
    if (std::isdigit(id[i])) {
      digit_begin = i;
    } else if (digit_begin >= 0) {
      digit_end = i;
      break;
    }
  }
  if (digit_begin >= 0) {
    Mark(mark_msg, std::stoi(id.substr(digit_begin, digit_end - digit_begin)));
  } else {
    Mark(mark_msg, -1);
  }
}

SteadyTimer::~SteadyTimer() {
  Stop();
  thread_.join();
}

void SteadyTimer::Start() {
  thread_ = std::thread([this] {
    QSetThreadName("SteadyTimer");
    timer_.expires_from_now(std::chrono::milliseconds(period_ms_));
    timer_.async_wait(std::bind(&SteadyTimer::Run, this));
    io_.run();
  });
}

void SteadyTimer::Stop() {
  if (!io_.stopped()) {
    io_.stop();
  }
}

void SteadyTimer::Run() {
  auto start = std::chrono::steady_clock().now();
  cb_();
  auto now = std::chrono::steady_clock().now();
  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
                  .count();
  if (diff > period_ms_) {
    LOG(ERROR) << "Callback cost more than period " << diff << " > "
               << period_ms_;
  }
  if (!one_shot_) {
    timer_.expires_from_now(std::chrono::milliseconds(period_ms_ - diff));
    timer_.async_wait(std::bind(&SteadyTimer::Run, this));
  }
}

}  // namespace qcraft
