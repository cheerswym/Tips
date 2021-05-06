#include "onboard/planner/planner_state.h"

#include <string>
#include <utility>

#include "google/protobuf/util/message_differencer.h"
#include "onboard/global/trace.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/planner_util.h"
#include "onboard/utils/proto_util.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {

void PlannerState::ClearTrajectories() {
  previous_trajectory_global.clear();
  previous_past_trajectory_global.clear();
}

void PlannerState::FromProto(const PlannerStateProto& proto) {
  header = proto.header();

  planner_frame_seq_num = proto.planner_frame_seq_num();

  for (const auto& e : proto.yellow_light_observations()) {
    yellow_light_observations[e.first].FromProto(e.second);
  }

  previous_trajectory_global.clear();
  for (const auto& e : proto.previous_trajectory_global()) {
    previous_trajectory_global.push_back(
        {.pos = Vec2dFromProto(e.pos()), .theta = e.theta()});
  }
  previous_trajectory = proto.previous_trajectory();

  previous_past_trajectory_global.clear();
  for (const auto& e : proto.previous_past_trajectory_global()) {
    previous_past_trajectory_global.push_back(
        {.pos = Vec2dFromProto(e.pos()), .theta = e.theta()});
  }

  stop_sign_states.clear();
  for (const auto& e : proto.stop_sign_states()) {
    stop_sign_states.emplace(e.first, qcraft::FromProto(e.second));
  }

  stop_sign_yield_list.clear();
  for (const auto& e : proto.stop_sign_yield_list()) {
    stop_sign_yield_list.emplace(
        e.first, absl::flat_hash_set<std::string>(e.second.id().begin(),
                                                  e.second.id().end()));
  }
  last_audio_alert_time = qcraft::FromProto(proto.last_audio_alert_time());

  parking_brake_release_time =
      qcraft::FromProto(proto.parking_brake_release_time());

  lane_change_state = proto.lane_change_state();

  planner_last_cycle_timeout = proto.planner_last_cycle_timeout();

  planner_skip_counter = proto.planner_skip_counter();

  input_seq_num = proto.input_seq_num();

  previous_autonomy_state = proto.previous_autonomy_state();

  previous_trajectory_plan_counter = proto.previous_trajectory_plan_counter();

  version = proto.version();

  last_door_override_time = proto.last_door_override_time();

  decider_state = proto.decider_state();
  initializer_state = proto.initializer_state();
  st_planner_trajectories = proto.st_planner_trajectories();
  current_time = absl::FromUnixMicros(proto.current_time());

  route_update_id = proto.route_update_id();

  // Freespace planner state.
  freespace_planner_state = proto.freespace_planner_state();

  mission_stage = proto.mission_stage();

  // Parallel planner state.
  // prev_target_lane_path restore later because it depends semantic map.
  // After map patch, it can restore here.
  prev_length_along_route = proto.prev_length_along_route();
  station_anchor.FromProto(proto.station_anchor());

  prev_route_sections =
      RouteSections::BuildFromProto(proto.prev_route_sections());

  // Reference line smooth.
  prev_smooth_state = proto.prev_smooth_state();

  // Prediction
  if (proto.has_prediction_state_proto()) {
    prediction_state.FromProto(proto.prediction_state_proto());
  }

  // Plan task queue
  for (const auto& task_proto : proto.plan_task_queue()) {
    plan_task_queue.emplace_back(task_proto);
  }

  stopped_at_route_end = proto.stopped_at_route_end();
}

// If FromProto/ToProto slow, use move instead
void PlannerState::ToProto(PlannerStateProto* proto) const {
  SCOPED_QTRACE("PlannerState::ToProto");

  proto->Clear();

  *proto->mutable_header() = header;

  proto->set_planner_frame_seq_num(planner_frame_seq_num);

  for (const auto& e : yellow_light_observations) {
    e.second.ToProto(&(*proto->mutable_yellow_light_observations())[e.first]);
  }

  for (const auto& e : previous_trajectory_global) {
    auto* point = proto->add_previous_trajectory_global();
    Vec2dToProto(e.pos, point->mutable_pos());
    point->set_theta(e.theta);
  }
  *proto->mutable_previous_trajectory() = previous_trajectory;

  for (const auto& e : previous_past_trajectory_global) {
    auto* point = proto->add_previous_past_trajectory_global();
    Vec2dToProto(e.pos, point->mutable_pos());
    point->set_theta(e.theta);
  }

  for (const auto& e : stop_sign_states) {
    qcraft::ToProto(e.second, &(*proto->mutable_stop_sign_states())[e.first]);
  }

  for (const auto& e : stop_sign_yield_list) {
    auto& id_list = (*proto->mutable_stop_sign_yield_list())[e.first];
    for (const auto& id : e.second) {
      id_list.add_id(id);
    }
  }

  qcraft::ToProto(last_audio_alert_time,
                  proto->mutable_last_audio_alert_time());

  if (parking_brake_release_time < absl::UnixEpoch()) {
    qcraft::ToProto(absl::UnixEpoch(),
                    proto->mutable_parking_brake_release_time());
  } else {
    qcraft::ToProto(parking_brake_release_time,
                    proto->mutable_parking_brake_release_time());
  }

  *proto->mutable_lane_change_state() = lane_change_state;

  proto->set_planner_last_cycle_timeout(planner_last_cycle_timeout);

  proto->set_planner_skip_counter(planner_skip_counter);

  *proto->mutable_input_seq_num() = input_seq_num;

  *proto->mutable_previous_autonomy_state() = previous_autonomy_state;

  proto->set_previous_trajectory_plan_counter(previous_trajectory_plan_counter);

  proto->set_version(version);

  proto->set_last_door_override_time(last_door_override_time);

  *proto->mutable_decider_state() = decider_state;

  *proto->mutable_initializer_state() = initializer_state;

  prev_lane_path_before_lc.ToProto(proto->mutable_prev_lane_path_before_lc());
  *proto->mutable_st_planner_trajectories() = st_planner_trajectories;
  proto->set_current_time(absl::ToUnixMicros(current_time));
  proto->set_route_update_id(route_update_id);
  *proto->mutable_freespace_planner_state() = freespace_planner_state;

  if (!planner_semantic_map_modifier.IsEmpty()) {
    *proto->mutable_planner_semantic_map_modifier() =
        PlannerSemanticMapModificationToProto(planner_semantic_map_modifier);
  }

  *proto->mutable_mission_stage() = mission_stage;

  // Parallel planner state.
  prev_target_lane_path.ToProto(proto->mutable_prev_target_lane_path());
  proto->set_prev_length_along_route(prev_length_along_route);
  station_anchor.ToProto(proto->mutable_station_anchor());
  prev_route_sections.ToProto(proto->mutable_prev_route_sections());

  // Reference line smooth.
  proto->set_prev_smooth_state(prev_smooth_state);
  proto->mutable_smooth_result_map()->mutable_lane_id_vec()->Reserve(
      smooth_result_map.smoothed_result_map().size());
  for (const auto& it : smooth_result_map.smoothed_result_map()) {
    auto* lane_id_vec = proto->mutable_smooth_result_map()->add_lane_id_vec();
    lane_id_vec->mutable_lane_id()->Add(it.first.begin(), it.first.end());
  }

  // For teleop lane change.
  preferred_lane_path.ToProto(proto->mutable_preferred_lane_path());

  // Prediction
  prediction_state.ToProto(proto->mutable_prediction_state_proto());

  // Plan task queue
  proto->mutable_plan_task_queue()->Reserve(plan_task_queue.size());
  int index = 0;
  for (const auto& task : plan_task_queue) {
    task.ToProto(proto->add_plan_task_queue(), index);
    index++;
  }

  proto->set_stopped_at_route_end(stopped_at_route_end);
}

// This `operator==` could not be defined in anonymous namespace.
bool operator==(const PlannerState::PosePoint& lhs,
                const PlannerState::PosePoint& rhs) {
  return lhs.pos.x() == rhs.pos.x() && lhs.pos.y() == rhs.pos.y() &&
         lhs.theta == rhs.theta;
}

bool PlannerState::operator==(const PlannerState& other) const {
  if (previous_trajectory_global != other.previous_trajectory_global ||
      previous_past_trajectory_global !=
          other.previous_past_trajectory_global ||
      previous_trajectory != other.previous_trajectory ||
      stop_sign_states != other.stop_sign_states ||
      stop_sign_yield_list != other.stop_sign_yield_list ||
      last_audio_alert_time != other.last_audio_alert_time ||
      parking_brake_release_time != other.parking_brake_release_time ||
      planner_skip_counter != other.planner_skip_counter ||
      !ProtoEquals(lane_change_state, other.lane_change_state) ||
      planner_last_cycle_timeout != other.planner_last_cycle_timeout ||
      !ProtoEquals(input_seq_num, other.input_seq_num) ||
      previous_autonomy_state != other.previous_autonomy_state ||
      previous_trajectory_plan_counter !=
          other.previous_trajectory_plan_counter ||
      last_door_override_time != other.last_door_override_time ||
      prev_lane_path_before_lc != other.prev_lane_path_before_lc ||
      !ProtoEquals(initializer_state, other.initializer_state) ||
      !ProtoEquals(st_planner_trajectories, other.st_planner_trajectories) ||
      !ProtoEquals(freespace_planner_state, other.freespace_planner_state) ||
      prev_target_lane_path != other.prev_target_lane_path ||
      prev_length_along_route != other.prev_length_along_route ||
      station_anchor != other.station_anchor ||
      preferred_lane_path != other.preferred_lane_path ||
      prev_route_sections != other.prev_route_sections ||
      prev_smooth_state != other.prev_smooth_state ||
      current_time != other.current_time) {
    return false;
  }
  // semantic map modifier
  if (planner_semantic_map_modifier.max_speed_limit !=
          other.planner_semantic_map_modifier.max_speed_limit ||
      planner_semantic_map_modifier.lane_speed_limit_map !=
          other.planner_semantic_map_modifier.lane_speed_limit_map) {
    return false;
  }

  return true;
}

bool PlannerState::Upgrade() {
  // trajectory_start_timestamp is set improperly, compatible for old runs";
  if (previous_trajectory.trajectory_start_timestamp() < 1E-6) {
    previous_trajectory.set_trajectory_start_timestamp(header.timestamp() /
                                                       1E6);
  }
  return true;
}

std::string PlannerState::DebugString() const {
  PlannerStateProto proto;
  ToProto(&proto);
  return proto.DebugString();
}

}  // namespace qcraft::planner
