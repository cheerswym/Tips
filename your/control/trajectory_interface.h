#ifndef ONBOARD_CONTROL_TRAJECTORY_INTERFACE_H_
#define ONBOARD_CONTROL_TRAJECTORY_INTERFACE_H_

#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "boost/circular_buffer.hpp"
#include "onboard/eval/qevent.h"
#include "onboard/math/vec.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::control {
class TrajectoryInterface {
 public:
  static constexpr double kTimeIncrement = 0.01;  // s.

  absl::Status Update(const AutonomyStateProto &autonomy_state,
                      const TrajectoryProto &planning_published_trajectory,
                      ControllerDebugProto *controller_debug_proto);

  // TODO(zhichao): modify API to separate query trajectory on transition and
  // original trajectory.
  ApolloTrajectoryPointProto QueryNearestTrajectoryPointByPositionWithSRange(
      bool on_transition_traj, double x, double y, double last_matched_point_s,
      double av_speed) const;

  ApolloTrajectoryPointProto QueryNearestTrajectoryPointByPosition(
      bool on_transition_traj, double x, double y) const;

  ApolloTrajectoryPointProto QueryTrajectoryPointByRelativeTime(
      bool on_transition_traj, double relative_time) const;

  // It is designed for lateral control. Allow to search a reference path
  // point beyond planner published trajectory range
  ApolloTrajectoryPointProto QueryTrajectoryPointBasedOnPathS(
      bool on_transition_traj, double s) const;

  const std::vector<ApolloTrajectoryPointProto> &GetAllTrajPoints(
      bool on_transition_traj) const;

  double GetPlannerStartTime(bool on_transition_traj) const;

  bool GetIsStationaryTrajectory() const { return is_stationary_trajectory_; }

  bool aeb_triggered() const { return aeb_triggered_; }

  bool GetIsLowSpeedFreespace() const { return is_low_speed_freespace_; }

 private:
  static constexpr int kAutoModeCacheSize = 20;  // 0.2 s.
  boost::circular_buffer<bool> auto_mode_cache_{kAutoModeCacheSize, false};

  void UpdateAllTrajPointsCurrent(
      double header_time, const TrajectoryProto &planning_published_trajectory);

  // Planner start time with the trajectory points.
  std::pair<double, std::vector<ApolloTrajectoryPointProto>>
      all_traj_points_current_;
  std::pair<double, std::vector<ApolloTrajectoryPointProto>>
      all_traj_points_previous_;
  std::pair<double, std::vector<ApolloTrajectoryPointProto>>
      all_traj_points_transition_;
  Chassis::GearPosition trajectory_gear_ = Chassis::GEAR_DRIVE;

  bool aeb_triggered_ = false;
  bool is_stationary_trajectory_ = false;
  bool is_low_speed_freespace_ = false;
  // Forward s in neighbour trajectory within a control cycle.
  double step_s_ = 0.0;
  double previous_trajectory_start_timestamp_ = 0.0;
  double planner_update_interval_ = 0.0;
};

}  // namespace qcraft::control

#endif  // ONBOARD_CONTROL_TRAJECTORY_INTERFACE_H_
