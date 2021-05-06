#include "onboard/planner/teleop_state.h"

#include <sstream>

#include "glog/logging.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {
namespace planner {

static bool Equals(const TeleopState& lhs, const TeleopState& rhs) {
  return lhs.request_case == rhs.request_case &&
         lhs.override_left_blinker == rhs.override_left_blinker &&
         lhs.override_left_blinker_on == rhs.override_left_blinker_on &&
         lhs.override_right_blinker == rhs.override_right_blinker &&
         lhs.override_right_blinker_on == rhs.override_right_blinker_on &&
         lhs.override_door == rhs.override_door &&
         lhs.override_door_open == rhs.override_door_open &&
         lhs.enable_traffic_light_stopping ==
             rhs.enable_traffic_light_stopping &&
         lhs.enable_lc_objects == rhs.enable_lc_objects &&
         lhs.brake_to_stop == rhs.brake_to_stop &&
         lhs.enable_pull_over == rhs.enable_pull_over;
}

void TeleopState::FromProto(const RemoteAssistToCarProto& proto) {
  request_case = proto.request_case();
  if (proto.has_left_blinker_override()) {
    const auto& blinker_override = proto.left_blinker_override();
    override_left_blinker = blinker_override.has_on();
    override_left_blinker_on = blinker_override.on();
  } else if (proto.has_right_blinker_override()) {
    const auto& blinker_override = proto.right_blinker_override();
    override_right_blinker = blinker_override.has_on();
    override_right_blinker_on = blinker_override.on();
  } else if (proto.has_emergency_blinker_override()) {
    const auto& blinker_override = proto.emergency_blinker_override();
    override_emergency_blinker = blinker_override.has_on();
    override_emergency_blinker_on = blinker_override.on();
  } else if (proto.has_door_override()) {
    override_door = proto.door_override().has_open();
    override_door_open = proto.door_override().open();
  } else if (proto.has_enable_feature_override()) {
    const auto& enable_feature_override = proto.enable_feature_override();
    if (enable_feature_override.has_enable_traffic_light_stopping()) {
      enable_traffic_light_stopping =
          enable_feature_override.enable_traffic_light_stopping();
      VLOG(1) << "Received override to "
              << (enable_traffic_light_stopping ? "enable" : "disable")
              << " stopping for traffic lights from teleop";
    }
    if (enable_feature_override.has_enable_lc_objects()) {
      enable_lc_objects = enable_feature_override.enable_lc_objects();
      VLOG(1) << "Received override to "
              << (enable_lc_objects ? "enable" : "disable")
              << " lane change avoid stationary objects";
    }
    if (enable_feature_override.has_enable_pull_over()) {
      enable_pull_over = enable_feature_override.enable_pull_over();
    }
  } else if (proto.has_driving_action_request()) {
    const auto& driving_action_request = proto.driving_action_request();
    if (driving_action_request.has_lane_change()) {
      queued_lane_change_requests.emplace_back(
          driving_action_request.lane_change());
      VLOG(1) << "Received lane change request: lane change direction "
              << LaneChangeRequestProto::Direction_Name(
                     driving_action_request.lane_change().direction());
    } else if (driving_action_request.has_rerouting()) {
      // TODO(xiang): move to RunMainLoop
      // route_manager->AddManualReroutingRequest(
      //     driving_action_request.rerouting());
      VLOG(1) << "Received rerouting request: "
              << driving_action_request.rerouting().DebugString();
    }
  } else if (proto.stop_vehicle().has_brake()) {
    brake_to_stop = proto.stop_vehicle().brake();
  }
}
/// \brief Important RemoteAssistToCarProto is only OneOf message
/// it cannot fill all data, only 1 message can filled.
void TeleopState::FillProto(RemoteAssistToCarProto* proto) const {
  QCHECK_NOTNULL(proto);
  switch (request_case) {
    case RemoteAssistToCarProto::kLeftBlinkerOverride: {
      auto left_override_blinker = proto->mutable_left_blinker_override();
      if (override_left_blinker) {
        left_override_blinker->set_on(override_left_blinker_on);
      } else {
        left_override_blinker->clear_on();
      }
    } break;
    case RemoteAssistToCarProto::kRightBlinkerOverride: {
      auto right_override_blinker = proto->mutable_right_blinker_override();
      if (override_right_blinker) {
        right_override_blinker->set_on(override_right_blinker_on);
      } else {
        right_override_blinker->clear_on();
      }
    } break;
    case RemoteAssistToCarProto::kDoorOverride:
      if (this->override_door) {
        auto door_override = proto->mutable_door_override();
        door_override->set_open(override_door_open);
        // TODO(xiang): last_door_override_time is set in planner_state.
        // last_door_override_time
      } else {
        proto->clear_door_override();
      }
      break;
    case RemoteAssistToCarProto::kEnableFeatureOverride: {
      auto enable_feature_override = proto->mutable_enable_feature_override();
      enable_feature_override->set_enable_traffic_light_stopping(
          enable_traffic_light_stopping);
      enable_feature_override->set_enable_lc_objects(enable_lc_objects);
      enable_feature_override->set_enable_pull_over(enable_pull_over);
    } break;
    case RemoteAssistToCarProto::kDrivingActionRequest: {
      // Deprecated.
    } break;
    case RemoteAssistToCarProto::kStopVehicle: {
      proto->mutable_stop_vehicle()->set_brake(brake_to_stop);
    } break;
    default:
      break;
  }
  return;
}

bool TeleopState::operator==(const TeleopState& rhs) const {
  return Equals(*this, rhs);
}

std::string TeleopState::DebugString() const {
  std::stringstream ss;
  ss << "request_case:" << request_case << '\n'
     << "override_left_blinker:" << override_left_blinker << '\n'
     << "override_left_blinker_on:" << override_left_blinker_on << '\n'
     << "override_right_blinker:" << override_right_blinker << '\n'
     << "override_right_blinker_on:" << override_right_blinker_on << '\n'
     << "override_door:" << override_door << '\n'
     << "override_door_open:" << override_door_open << '\n'
     << "enable_traffic_light_stopping:" << enable_traffic_light_stopping
     << '\n'
     << "enable_lc_objects:" << enable_lc_objects << '\n'
     << "brake_to_stop:" << brake_to_stop << '\n'
     << "enable_pull_over:" << enable_pull_over;
  return ss.str();
}

}  // namespace planner
}  // namespace qcraft
