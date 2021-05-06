#ifndef ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_
#define ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_

#include <string>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed_profile.h"

namespace qcraft {
namespace planner {

void ApplySpeedConstraintForReferenceSpeedProfile(
    double s_start, double s_end, double v_max,
    const std::vector<double> &path_s, std::vector<double> *v_s);

std::vector<double> CreateSpeedProfileWithConstraints(
    double v_now, const DrivePassage &drive_passage,
    const PlannerSemanticMapManager &psmm,
    absl::Span<const ConstraintProto::SpeedRegionProto> speed_zones,
    absl::Span<const ConstraintProto::StopLineProto> stop_points);

SpeedProfile IntegrateSpeedProfile(const DrivePassage &drive_passage,
                                   const std::vector<double> &v_s);

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements);

void UpdateDecisionConstraintDebugInfo(const ConstraintManager &constraint_mgr,
                                       ConstraintProto *constraint);

// Decide whether to enable initializer to decide lane change target.
bool EnableInitializerLaneChangeTargetDecision(
    const LaneChangeStage &lane_change_stage, const PathSlBoundary &sl_boundary,
    const FrenetBox &av_frenet_box);

// TODO(jiayu): implement this function:
// ConstraintProto::LeadingObjectProto ProjectTrajectorySTOnDrivePassage()

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_
