#ifndef ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_

#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"

namespace qcraft {
namespace planner {

struct SpeedConstraintGeneratorInput {
  const std::vector<StBoundaryWithDecision>* st_boundaries_with_decision =
      nullptr;
  const StGraph* st_graph = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const DiscretizedPath* path = nullptr;
  const DrivePassage* drive_passage = nullptr;
  const PathSlBoundary* path_sl_boundary = nullptr;
  const SpeedFinderParamsProto::SpeedDeciderParamsProto* speed_decider_params =
      nullptr;
};

void GenerateSpeedConstraints(const SpeedConstraintGeneratorInput& input,
                              ConstraintManager* constraint_mgr);

}  // namespace planner

}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_CONSTRAINT_GENERATOR_H_
