#ifndef ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_
#define ONBOARD_PLANNER_DECISION_DECIDER_OUTPUT_H_

#include <vector>

#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/crosswalk_state.pb.h"

namespace qcraft::planner {

struct CrosswalkDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> speed_regions;
  std::vector<CrosswalkStateProto> crosswalk_states;
};

struct TrafficLightDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  TrafficLightDeciderStateProto traffic_light_decider_state;
};
struct DeciderOutput {
  ConstraintManager constraint_manager;
  DeciderStateProto decider_state;
  bool initializer_lc_multiple_traj = false;
};

}  // namespace qcraft::planner

#endif
