#ifndef ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
#define ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_

#include "onboard/maps/lane_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<ConstraintProto::SpeedRegionProto>
BuildLcEndOfCurrentLaneConstraints(const mapping::LanePath &lane_path,
                                   const DrivePassage &dp, double ego_v);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_LC_END_OF_CURRENT_LANE_CONSTRAINT_H_
