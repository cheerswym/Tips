#ifndef ONBOARD_PLANNER_DECISION_TOLL_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TOLL_DECIDER_H_

#include <vector>

#include "onboard/planner/router/drive_passage.h"

namespace qcraft {

namespace planner {
absl::StatusOr<std::vector<ConstraintProto::SpeedRegionProto>>
BuildTollConstraints(const PlannerSemanticMapManager& psmm,
                     const DrivePassage& passage);
}
}  // namespace qcraft

#endif
