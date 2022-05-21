#include "onboard/utils/network_card_monitor.h"

#include <iostream>
#include <string>

#include "absl/strings/match.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "boost/algorithm/string.hpp"
#include "boost/process.hpp"

namespace qcraft {
NetworkCardMonitor::NetworkCardMonitor() {
  last_up_query_timestamp_ = absl::ToUnixMillis(absl::Now());
  last_network_up_package_bytes_ = CurrentUpPackageBytes();
}

int64 NetworkCardMonitor::CurrentUpSpeed() {
  const int64 current_timestamp = absl::ToUnixMillis(absl::Now());
  const auto current_network_package_bytes = CurrentUpPackageBytes();
  std::cout << current_network_package_bytes << std::endl;
  const int64 current_speed =
      (current_network_package_bytes - last_network_up_package_bytes_) /
      (current_timestamp - last_up_query_timestamp_) * 1000;
  last_network_up_package_bytes_ = current_network_package_bytes;
  last_up_query_timestamp_ = current_timestamp;
  return current_speed;
}

int64 NetworkCardMonitor::CurrentUpPackageBytes() {
  int64 current_package_bytes = 0;
  bool network_card_find = false;
  boost::process::ipstream out;
  boost::process::system("ifconfig", boost::process::std_out > out,
                         boost::process::std_err > stderr);
  for (std::string line; std::getline(out, line);) {
    if (boost::starts_with(line, "enp") || boost::starts_with(line, "wlp")) {
      network_card_find = true;
    }
    if (network_card_find && absl::StrContains(line, "TX packets")) {
      try {
        network_card_find = false;
        int start_index = line.find("bytes") + 6;
        int end_index = line.find("(") - 1;
        current_package_bytes +=
            std::stoll(line.substr(start_index, end_index).c_str());
      } catch (std::exception &e) {
        return 0;
      }
    }
  }
  return current_package_bytes;
}
}  // namespace qcraft
