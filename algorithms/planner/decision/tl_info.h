#ifndef ONBOARD_PLANNER_DECISION_TL_INFO_H_
#define ONBOARD_PLANNER_DECISION_TL_INFO_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"

namespace qcraft {
namespace planner {

struct SingleTlInfo {
  mapping::ElementId tl_id;
  TrafficLightState tl_state;
  double estimated_turn_red_time_left;
};

class TlInfo {
 public:
  TlInfo(mapping::ElementId lane_id,
         const std::vector<double>& control_point_relative_s,
         bool can_go_on_red,
         const absl::flat_hash_map<TrafficLightDirection, SingleTlInfo>& tls,
         bool is_fresh, const std::string& last_error_msg);

  mapping::ElementId lane_id() const { return lane_id_; }
  bool can_go_on_red() const { return can_go_on_red_; }

  const std::vector<double>& control_point_relative_s() const {
    return control_point_relative_s_;
  }

  const absl::flat_hash_map<TrafficLightDirection, SingleTlInfo>& tls() const {
    return tls_;
  }
  TrafficLightControlType tl_control_type() const { return tl_control_type_; }

  bool is_fresh() const { return is_fresh_; }
  const std::string& last_error_msg() const { return last_error_msg_; }
  bool empty() const { return is_empty_; }

  std::string DebugString() const;

  void ToProto(TrafficLightInfoProto* proto) const;

 private:
  // lane properties
  mapping::ElementId lane_id_;
  std::vector<double> control_point_relative_s_;
  bool can_go_on_red_;
  // associated tl properties
  absl::flat_hash_map<TrafficLightDirection, SingleTlInfo> tls_;
  TrafficLightControlType tl_control_type_ =
      TrafficLightControlType::SINGLE_DIRECTION;
  // freshness properties
  bool is_fresh_ = false;
  std::string last_error_msg_;
  bool is_empty_ = true;
};

using TrafficLightInfoMap = absl::flat_hash_map<mapping::ElementId, TlInfo>;

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_TL_INFO_H_
