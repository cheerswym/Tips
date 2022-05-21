#include "onboard/utils/lite_msg_stats.h"

#include <algorithm>

#include "onboard/utils/time_util.h"

namespace {
constexpr int64_t kTenSeconds = 10000000; /* in us */
}  // namespace

namespace qcraft {

LiteMsgStats::LiteMsgStats(const std::string &topic,
                           const std::string &msg_type)
    : msg_name_(topic), msg_type_(msg_type), entries_(kWindowSize) {}

void LiteMsgStats::GotNewMessage(int64_t stamp, uint32_t size) {
  if (!entries_.empty()) {
    if (stamp - entries_.back().timestamp > kTenSeconds) {
      Clear();
    } else if (entries_.size() == kWindowSize) {
      first_timestamp_ = entries_[0].timestamp;
    }
  }
  entries_.push_back({.timestamp = stamp, .size = size});
}

double LiteMsgStats::AvgMsgSize() const {
  if (entries_.empty()) {
    return 0;
  }
  auto size_sum = [](uint32_t lhs, const MsgEnt &rhs) {
    return lhs + rhs.size;
  };
  uint32_t sum =
      std::accumulate(entries_.begin(), entries_.end(), 0U, size_sum);
  return static_cast<double>(sum) / entries_.size();
}

double LiteMsgStats::Hz() const {
  int n = entries_.size();
  if (n <= 1) {
    return 0.0;
  }

  int64_t time_elapsed = 0;
  if (first_timestamp_ < 0) {
    time_elapsed = entries_[n - 1].timestamp - entries_[0].timestamp;
    n = n - 1;
  } else {
    time_elapsed = entries_[n - 1].timestamp - first_timestamp_;
  }

  // n messages in time_elapsed * 1e-6 seconds
  double freq = n * 1e6 / time_elapsed;
  // It is possible that message traffic boosts in a short period then goes
  // silent, this causes Hz rocket into the sky. We add a constraint here to
  // remedy
  return std::clamp(freq, 0.0, kFreqMax);
}

bool LiteMsgStats::IsStale() const {
  if (!entries_.empty()) {
    auto delta =
        ToUnixDoubleSeconds(absl::Now()) - entries_.back().timestamp * 1e-6;
    return delta > 2 / this->Hz();
  }
  return false;
}

LiteMsgStats::Stats LiteMsgStats::GetStats() const {
  return {
      msg_name_,    msg_type_, entries_.empty() ? 0 : entries_.back().timestamp,
      AvgMsgSize(), Hz(),      IsStale()};
}

void LiteMsgStats::Clear() {
  entries_.clear();
  first_timestamp_ = -1;
}

}  // namespace qcraft
