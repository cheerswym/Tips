#ifndef ONBOARD_UTILS_NETWORK_CARD_MONITOR_H_
#define ONBOARD_UTILS_NETWORK_CARD_MONITOR_H_

#include "onboard/base/integral_types.h"

namespace qcraft {
class NetworkCardMonitor {
 public:
  // constructor
  NetworkCardMonitor();
  int64 CurrentUpSpeed();

 private:
  int64 CurrentUpPackageBytes();

 private:
  int64 last_up_query_timestamp_;
  int64 last_network_up_package_bytes_;
};

}  // namespace qcraft
#endif  // ONBOARD_UTILS_NETWORK_CARD_MONITOR_H_
