#ifndef ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_
#define ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_

#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

absl::StatusOr<DrivePassage> BuildDrivePassage(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& lane_path_from_pose,
    const mapping::LanePoint& anchor_point, double planning_horizon,
    double keep_behind_len, bool all_lanes_virtual = false);

// This is only used in the prediction module.
DrivePassage BuildDrivePassageFromLanePath(const SemanticMapManager& smm,
                                           const mapping::LanePath& lane_path,
                                           double step_s, bool avoid_loop);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_ROUTER_DRIVE_PASSAGE_BUILDER_H_
