#ifndef ONBOARD_PLANNER_SPEED_ST_GRAPH_DATA_H_
#define ONBOARD_PLANNER_SPEED_ST_GRAPH_DATA_H_

#include <vector>

#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/st_boundary.h"

namespace qcraft::planner {

class StGraphData {
 public:
  StGraphData(const SpeedLimit& speed_limit, double cruise_speed,
              double path_length, double total_time)
      : speed_limit_(speed_limit),
        cruise_speed_(cruise_speed),
        path_length_(path_length),
        total_time_(total_time) {}

  const SpeedLimit& speed_limit() const { return speed_limit_; }

  double cruise_speed() const { return cruise_speed_; }

  double path_length() const { return path_length_; }

  double total_time() const { return total_time_; }

 private:
  SpeedLimit speed_limit_;
  double cruise_speed_ = 0.0;
  double path_length_ = 0.0;
  double total_time_ = 0.0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_GRAPH_DATA_H_
