#ifndef ONBOARD_VIS_QVIEW_ROUTE_INFO_H_
#define ONBOARD_VIS_QVIEW_ROUTE_INFO_H_

#include <iostream>
#include <string>
#include <vector>

#include "onboard/base/integral_types.h"
namespace qcraft {

class QviewRouteInfo {
 private:
  struct Station {
    int id_;
    std::string city_;
    std::string name_;
    double longtitude_;
    double latitude_;
    double altitude_;
    double heading_;
  };

 public:
  QviewRouteInfo();
  bool Parse(const std::string &route_info);

 public:
  void Clear();

  bool has_route_info_;
  std::string uid_;
  std::string city_;
  std::string name_;
  bool is_loop_;
  std::string service_start_time_;
  std::string service_stop_time_;
  std::string service_break_time_;
  int64 update_timestamp_;
  std::vector<Station> stations_;
};

}  // namespace qcraft

#endif
