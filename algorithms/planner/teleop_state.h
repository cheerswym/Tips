#ifndef ONBOARD_PLANNER_TELEOP_STATE_H_
#define ONBOARD_PLANNER_TELEOP_STATE_H_

#include <deque>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/remote_assist.pb.h"

namespace qcraft {
namespace planner {
struct TeleopState {
  // Blinker override. If X_override is true, the blinker on the corresponding
  // side is forced on/off according to X_override_on. If not, they are decided
  // by planner's logic.
  bool override_left_blinker = false;
  bool override_left_blinker_on = false;
  bool override_right_blinker = false;
  bool override_right_blinker_on = false;
  bool override_emergency_blinker = false;
  bool override_emergency_blinker_on = false;
  std::optional<LaneChangeRequestProto::Direction> lane_change_direction;

  // Door override. If override_door is true, the door is forced open/close
  // according to override_door_open. Door opening controlled by planner has not
  // yet been implemented.
  bool override_door = false;
  bool override_door_open = false;
  // double last_door_override_time = 0.0; // Now in planner state

  // Traffic light stopping enable status.
  bool enable_traffic_light_stopping = true;
  // Lane change avoid stationary objects
  bool enable_lc_objects = true;

  // Brake the ego vehicle, set to negative to clear the state.
  double brake_to_stop = -1.0;

  bool enable_pull_over = false;

  RemoteAssistToCarProto::RequestCase request_case =
      RemoteAssistToCarProto::REQUEST_NOT_SET;
  // DrivingActionRequestProto drive_action_request;
  // ------ replace this queues below with the above members in future ------

  // Pending driver actions (buttons on the steering wheel).
  std::queue<DriverAction> pending_driver_actions = {};
  // Driving action requests being executed.
  std::deque<LaneChangeRequestProto> queued_lane_change_requests = {};

  void FromProto(const RemoteAssistToCarProto& remote_assist_to_car_proto);
  void FillProto(RemoteAssistToCarProto* remote_assist_to_car_proto) const;
  bool operator==(const TeleopState& rhs) const;
  bool operator!=(const TeleopState& rhs) const { return !(*this == rhs); }

  void Clear() {
    TeleopState default_state{};
    *this = default_state;
  }

  void ClearPendingQueue() {
    pending_driver_actions = {};
    queued_lane_change_requests = {};
  }

  std::string DebugString() const;

  // <<Delegate TeleOp state begin
  bool IsOverrideLeftBlinker() const { return override_left_blinker; }

  void SetOverrideLeftBlinker(bool override_left_blinker) {
    this->override_left_blinker = override_left_blinker;
  }

  bool IsOverrideLeftBlinkerOn() const { return override_left_blinker_on; }

  void SetOverrideLeftBlinkerOn(bool override_left_blinker_on) {
    this->override_left_blinker_on = override_left_blinker_on;
  }

  bool IsOverrideRightBlinker() const { return override_right_blinker; }

  void SetOverrideRightBlinker(bool override_right_blinker) {
    this->override_right_blinker = override_right_blinker;
  }

  bool IsOverrideRightBlinkerOn() const { return override_right_blinker_on; }

  void SetOverrideRightBlinkerOn(bool override_right_blinker_on) {
    this->override_right_blinker_on = override_right_blinker_on;
  }

  bool IsOverrideEmergencyBlinker() const { return override_emergency_blinker; }

  void SetOverrideEmergencyBlinker(bool override_emergency_blinker) {
    this->override_emergency_blinker = override_emergency_blinker;
  }

  bool IsOverrideEmergencyBlinkerOn() const {
    return override_emergency_blinker_on;
  }

  void SetOverrideEmergencyBlinkerOn(bool override_emergency_blinker_on) {
    this->override_emergency_blinker_on = override_emergency_blinker_on;
  }

  bool IsOverrideDoor() const { return override_door; }

  void SetOverrideDoor(bool override_door) {
    this->override_door = override_door;
  }

  bool IsOverrideDoorOpen() const { return override_door_open; }

  void SetOverrideDoorOpen(bool override_door_open) {
    this->override_door_open = override_door_open;
  }

  bool IsEnableTrafficLightStopping() const {
    return enable_traffic_light_stopping;
  }

  void SetEnableTrafficLightStopping(bool enable_traffic_light_stopping) {
    this->enable_traffic_light_stopping = enable_traffic_light_stopping;
  }

  bool IsEnableLcObjects() const { return enable_lc_objects; }

  void SetEnableLcObjects(bool enable_lc_objects) {
    this->enable_lc_objects = enable_lc_objects;
  }

  double BrakeToStop() const { return brake_to_stop; }

  void SetBrakeToStop(double brake_to_stop) {
    this->brake_to_stop = brake_to_stop;
  }

  bool IsEnablePullOver() const { return enable_pull_over; }

  void SetEnablePullOver(bool enable_pull_over) {
    this->enable_pull_over = enable_pull_over;
  }

  void AddDriveAction(const DriverAction& drive_action) {
    return pending_driver_actions.push(drive_action);
  }
  const std::queue<DriverAction>& GetPendingDriverActions() const {
    return pending_driver_actions;
  }
};
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_TELEOP_STATE_H_
