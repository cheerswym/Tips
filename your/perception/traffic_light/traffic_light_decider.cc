#include "onboard/perception/traffic_light/traffic_light_decider.h"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"

namespace qcraft {

bool TrafficLightDecider::TlVote::operator>(const TlVote& rhs_tl_vote) const {
  // Trust state with high belief.
  if (this->belief < rhs_tl_vote.belief) {
    return false;
  } else if (this->belief > rhs_tl_vote.belief) {
    return true;
  } else {
    // Draw beief: red > yellow > flashing green >
    // green > flashing yellow > unknown.
    if (this->tl_state == TlState::TL_STATE_RED) {
      return true;
    } else if (this->tl_state == TlState::TL_STATE_YELLOW &&
               rhs_tl_vote.tl_state != TlState::TL_STATE_RED) {
      return true;
    } else if (this->tl_state == TlState::TL_STATE_GREEN_FLASHING &&
               (rhs_tl_vote.tl_state != TlState::TL_STATE_YELLOW &&
                rhs_tl_vote.tl_state != TlState::TL_STATE_RED)) {
      return true;
    } else if (this->tl_state == TlState::TL_STATE_GREEN &&
               (rhs_tl_vote.tl_state != TlState::TL_STATE_GREEN_FLASHING &&
                rhs_tl_vote.tl_state != TlState::TL_STATE_YELLOW &&
                rhs_tl_vote.tl_state != TlState::TL_STATE_RED)) {
      return true;
    } else if (this->tl_state == TlState::TL_STATE_YELLOW_FLASHING &&
               (rhs_tl_vote.tl_state != TlState::TL_STATE_GREEN &&
                rhs_tl_vote.tl_state != TlState::TL_STATE_GREEN_FLASHING &&
                rhs_tl_vote.tl_state != TlState::TL_STATE_YELLOW &&
                rhs_tl_vote.tl_state != TlState::TL_STATE_RED)) {
      return true;
    } else {
      return false;
    }
  }
}

void TrafficLightDecider::VoteAcrossMultipleCameras(
    TrafficLightStatesProto* traffic_light_states) {
  SCOPED_QTRACE("TrafficLightDecider::VoteAcrossMultipleCameras");
  // traffic light id ==> camera indices
  absl::flat_hash_map<mapping::ElementId, std::vector<int>>
      traffic_light_2_cameras;
  for (int i = 0; i < traffic_light_states->states_size(); ++i) {
    const auto& traffic_light_state = traffic_light_states->states(i);
    const auto traffic_light_id = traffic_light_state.traffic_light_id();
    // Use raw index instead of camera_id.
    traffic_light_2_cameras[traffic_light_id].emplace_back(i);
  }
  // Get the max vote from single traffic light.
  const auto get_single_tl_max_vote =
      [this, &traffic_light_states,
       &traffic_light_2_cameras](mapping::ElementId tl_id) -> TlState {
    const auto& camera_indices = traffic_light_2_cameras[tl_id];
    absl::flat_hash_map<TlState, int> camera_votes;
    for (const auto& state_i : camera_indices) {
      auto traffic_light_state = traffic_light_states->states(state_i);
      TlState tl_state = TrafficLightStateProto2TlState(traffic_light_state);
      camera_votes[tl_state] += 1;
    }
    TlVote max_vote{.tl_state = TlState::TL_STATE_UNKNOWN, .belief = 0};
    for (auto it = camera_votes.begin(); it != camera_votes.end(); ++it) {
      auto vote = TlVote{.tl_state = it->first, .belief = it->second};
      if (vote > max_vote) {
        max_vote = vote;
      }
    }
    return max_vote.tl_state;
  };
  // traffic light id ==> max vote state
  absl::flat_hash_map<mapping::ElementId, TlState> traffic_light_2_max_vote;
  for (const auto& t : traffic_light_2_cameras) {
    const auto traffic_light_id = t.first;
    const auto max_vote = get_single_tl_max_vote(traffic_light_id);
    traffic_light_2_max_vote[traffic_light_id] = max_vote;
  }
  // Sync the state with raw proto in place.
  for (int i = 0; i < traffic_light_states->states_size(); ++i) {
    auto traffic_light_state_pointer = traffic_light_states->mutable_states(i);
    auto traffic_light_id = traffic_light_state_pointer->traffic_light_id();
    auto max_vote = traffic_light_2_max_vote[traffic_light_id];
    UpdateTrafficLightStateProto(max_vote, traffic_light_state_pointer);
  }
}

TrafficLightDecider::TlState
TrafficLightDecider::TrafficLightStateProto2TlState(
    const TrafficLightStateProto& proto) {
  if (proto.color() == TL_UNKNOWN) return TlState::TL_STATE_UNKNOWN;
  if (proto.color() == TL_RED) return TlState::TL_STATE_RED;
  if (proto.color() == TL_GREEN && !proto.flashing()) {
    return TlState::TL_STATE_GREEN;
  }
  if (proto.color() == TL_GREEN && proto.flashing()) {
    return TlState::TL_STATE_GREEN_FLASHING;
  }
  if (proto.color() == TL_YELLOW && !proto.flashing()) {
    return TlState::TL_STATE_YELLOW;
  }
  if (proto.color() == TL_YELLOW && proto.flashing()) {
    return TlState::TL_STATE_YELLOW_FLASHING;
  }
  return TlState::TL_STATE_UNKNOWN;
}

void TrafficLightDecider::UpdateTrafficLightStateProto(
    const TlState& max_vote, TrafficLightStateProto* proto) {
  proto->set_temporal_fused_color(proto->color());
  switch (max_vote) {
    case TlState::TL_STATE_UNKNOWN: {
      proto->set_color(TL_UNKNOWN);
      break;
    }
    case TlState::TL_STATE_RED: {
      proto->set_color(TL_RED);
      break;
    }
    case TlState::TL_STATE_GREEN: {
      proto->set_temporal_fused_flashing(proto->flashing());
      proto->set_color(TL_GREEN);
      proto->set_flashing(false);
      break;
    }
    case TlState::TL_STATE_GREEN_FLASHING: {
      proto->set_temporal_fused_flashing(proto->flashing());
      proto->set_color(TL_GREEN);
      proto->set_flashing(true);
      break;
    }
    case TlState::TL_STATE_YELLOW: {
      proto->set_temporal_fused_flashing(proto->flashing());
      proto->set_color(TL_YELLOW);
      proto->set_flashing(false);
      break;
    }
    case TlState::TL_STATE_YELLOW_FLASHING: {
      proto->set_temporal_fused_flashing(proto->flashing());
      proto->set_color(TL_YELLOW);
      proto->set_flashing(true);
      break;
    }
    default: {
      QLOG(FATAL) << "Unexpected traffic light state.";
    }
  }
}

}  // namespace qcraft
