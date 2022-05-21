#ifndef ONBOARD_PERCEPTION_TRAFFIC_LIGHT_TRAFFIC_LIGHT_DECIDER_H_
#define ONBOARD_PERCEPTION_TRAFFIC_LIGHT_TRAFFIC_LIGHT_DECIDER_H_

#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {

class TrafficLightDecider {
 public:
  void VoteAcrossMultipleCameras(TrafficLightStatesProto* traffic_light_states);

 private:
  // Sync traffic light state with planner.
  enum class TlState {
    TL_STATE_UNKNOWN = 0,
    TL_STATE_RED = 1,
    TL_STATE_YELLOW = 2,
    TL_STATE_GREEN_FLASHING = 3,
    TL_STATE_GREEN = 4,
    TL_STATE_YELLOW_FLASHING = 5,
  };
  struct TlVote {
    TlState tl_state;
    int belief;
    bool operator>(const TlVote& rhs_tl_vote) const;
  };
  TlState TrafficLightStateProto2TlState(const TrafficLightStateProto& proto);
  void UpdateTrafficLightStateProto(const TlState& max_vote,
                                    TrafficLightStateProto* proto);
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_TRAFFIC_LIGHT_TRAFFIC_LIGHT_DECIDER_H_
