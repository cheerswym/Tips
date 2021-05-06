#ifndef ONBOARD_PLANNER_SCHEDULER_SCHEDULER_OUTPUT_H_
#define ONBOARD_PLANNER_SCHEDULER_SCHEDULER_OUTPUT_H_

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"

namespace qcraft::planner {

struct ClearanceCheckOutput {
  std::vector<Vec2d> pseudo_traj;

  enum ObjectDecision {
    LC_IRRELEVANT = -1,
    LC_UNSAFE = 0,
    LC_FOLLOW = 1,
    LC_OVERTAKE = 2,
    LC_LEAD = 3
  };
  // The id is trajectory id.
  absl::flat_hash_map<std::string, ObjectDecision> objects_decision;
};

struct SchedulerOutput {
  bool is_fallback = false;
  bool is_expert = false;
  // TODO(weijun): Move drive_passage.h to scheduler folder.
  DrivePassage drive_passage;
  // l-s boundaries on drive passage center.
  PathSlBoundary sl_boundary;
  LaneChangeStateProto lane_change_state;
  // For constraints on lane change pause stage.
  mapping::LanePath lane_path_before_lc;
  double length_along_route = std::numeric_limits<double>::max();
  bool should_smooth = false;
  bool borrow_lane = false;
  FrenetBox av_frenet_box_on_drive_passage;

  std::vector<std::string> reasons;
  std::optional<ClearanceCheckOutput> clearance_output = std::nullopt;
  // TODO(weijun): add constraints
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_SCHEDULER_OUTPUT_H_
