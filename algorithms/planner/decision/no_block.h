#ifndef ONBOARD_PLANNER_DECISION_NO_BLOCK_H_
#define ONBOARD_PLANNER_DECISION_NO_BLOCK_H_

#include <vector>

#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// This function return no block constraint. For more details see 'no_block.md'
std::vector<ConstraintProto::SpeedRegionProto> BuildNoBlockConstraints(
    const PlannerSemanticMapManager &planner_semantic_map_manager,
    const DrivePassage &passage);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_NO_BLOCK_H_
