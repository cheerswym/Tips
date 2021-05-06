#ifndef ONBOARD_PLANNER_PLANNER_STATE_H_
#define ONBOARD_PLANNER_PLANNER_STATE_H_

#include <deque>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/decision/traffic_light_info_collector.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/plan/plan_task.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/prediction/container/prediction_state.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {
// The cross iteration states of planner.

struct PlannerState {
  LiteHeader header;

  int planner_frame_seq_num;

  // TODO(jiayu): Rename this struct name, after delete tl_history manager.
  YellowLightObservationsNew yellow_light_observations;

  // Global coordinates of previous planned trajectory.
  struct PosePoint {
    Vec2d pos;
    double theta = 0.0;
  };
  std::vector<PosePoint> previous_trajectory_global;
  std::vector<PosePoint> previous_past_trajectory_global;
  // Previous planned trajectory.
  TrajectoryProto previous_trajectory;

  // Stop signs, element ID to first full stop time.
  // TODO(lidong): Rename to stopsign_stop_duration.
  absl::flat_hash_map<mapping::ElementId, absl::Time> stop_sign_states;
  // Stop sign element ID to list of objects arriving before we did at this
  // stop sign.
  absl::flat_hash_map<mapping::ElementId, absl::flat_hash_set<std::string>>
      stop_sign_yield_list;

  // Audio playing
  absl::Time last_audio_alert_time;

  // Paking brake release time.
  absl::Time parking_brake_release_time;

  LaneChangeStateProto lane_change_state;

  bool planner_last_cycle_timeout = false;

  // Record how many loops were skipped between this and previous trajectory.
  int planner_skip_counter = 0;

  InputSeqNum input_seq_num;

  AutonomyStateProto previous_autonomy_state;

  int previous_trajectory_plan_counter = 0;

  int version = 4;  // When snapshot upgrade, change version

  double last_door_override_time = 0.0;

  absl::Time current_time;

  int64_t route_update_id = -1;

  // Clear trajectory related planner states.
  void ClearTrajectories();

  void FromProto(const PlannerStateProto& proto);

  void ToProto(PlannerStateProto* proto) const;

  bool Upgrade();  // Used for snapshot version compatible

  bool operator==(const PlannerState& other) const;

  bool operator!=(const PlannerState& other) const { return !(*this == other); }

  std::string DebugString() const;

  // Freespace planner state.
  FreespacePlannerStateProto freespace_planner_state;

  // ------------- Planner 3.0 -----------------
  mapping::LanePath prev_lane_path_before_lc;
  // State of decider
  DeciderStateProto decider_state;
  // State of initializer
  InitializerStateProto initializer_state;
  // Spacetime planner object
  SpacetimePlannerTrajectories st_planner_trajectories;

  // ---------------- Planner 3.5 --------------
  // ---------- Multiple trajectories ----------

  // Previous target lane path starting from the position that is slightly
  // behind plan start point: (1) Insure successful projection as plan start
  // point might jump backwards due to resetting. (2) Keep decision
  // consistency.
  mapping::LanePath prev_target_lane_path;
  double prev_length_along_route = std::numeric_limits<double>::max();
  // For stabilization of drive passage stations across frames.
  mapping::LanePoint station_anchor;

  // For teleop lane change.
  mapping::LanePath preferred_lane_path;

  // Route sections starting from the point that is several meters behind plan
  // start point.
  RouteSections prev_route_sections;

  // Reference line smooth.
  bool prev_smooth_state = false;

  // Planner semantic map manager modifier.
  PlannerSemanticMapModification planner_semantic_map_modifier;

  MissionStageProto mission_stage;

  std::deque<PlanTask> plan_task_queue;

  // Prediction
  prediction::PredictionState prediction_state;

  SmoothedReferenceLineResultMap smooth_result_map;

  bool stopped_at_route_end = false;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLANNER_STATE_H_
