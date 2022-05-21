#ifndef ONBOARD_UTILS_LITE_MSG_STATS_H_
#define ONBOARD_UTILS_LITE_MSG_STATS_H_

// utility to show lite message statistics

#include <cstdint>
#include <numeric>
#include <string>

#include "boost/circular_buffer.hpp"

namespace qcraft {

class LiteMsgStats {
 public:
  static constexpr double kFreqMax = 100000.0; /*10us*/
  static constexpr int kWindowSize = 5;
  static_assert(kWindowSize >= 2);

 public:
  LiteMsgStats() = default;

  LiteMsgStats(const std::string &topic, const std::string &msg_type);

  const std::string &TopicName() const { return msg_name_; }
  const std::string &MsgType() const { return msg_type_; }

  void GotNewMessage(int64_t stamp, uint32_t size);

  double AvgMsgSize() const;

  double Hz() const;

  bool IsStale() const;
  void Clear();

 public:
  struct Stats {
    std::string msg_name;
    std::string msg_type;
    int64_t last_stamp;
    double avg_msg_size;
    double hz;
    bool is_stale;
  };
  Stats GetStats() const;

 private:
  struct MsgEnt {
    int64_t timestamp; /* in us */
    uint32_t size;
  };

  std::string msg_name_, msg_type_;
  boost::circular_buffer<MsgEnt> entries_;
  int64_t first_timestamp_ = -1;
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_LITE_MSG_STATS_H_
