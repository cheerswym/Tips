#include "onboard/planner/speed/constraint_generator.h"

#include <utility>

#include "onboard/lite/logging.h"
#include "onboard/planner/speed/decider/close_object_slowdown_decider.h"

namespace qcraft {
namespace planner {

void GenerateSpeedConstraints(const SpeedConstraintGeneratorInput& input,
                              ConstraintManager* constraint_mgr) {
  QCHECK_NOTNULL(input.st_boundaries_with_decision);
  QCHECK_NOTNULL(input.st_graph);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.path);
  QCHECK_NOTNULL(input.drive_passage);
  QCHECK_NOTNULL(input.path_sl_boundary);
  QCHECK_NOTNULL(input.speed_decider_params);

  constexpr double kCloseObjectSlowdownMaxDistance = 2.0;  // m

  std::vector<CloseSpaceTimeObject> close_spacetime_objects;
  if (input.speed_decider_params->enable_stationary_close_object_slowdown()) {
    close_spacetime_objects = input.st_graph->GetCloseSpaceTimeObjects(
        *input.st_boundaries_with_decision,
        input.st_traj_mgr->stationary_object_trajs(),
        kCloseObjectSlowdownMaxDistance);
  }
  if (input.speed_decider_params->enable_moving_close_object_slowdown()) {
    auto moving_close_spacetime_objects =
        input.st_graph->GetCloseSpaceTimeObjects(
            *input.st_boundaries_with_decision,
            input.st_traj_mgr->moving_object_trajs(),
            kCloseObjectSlowdownMaxDistance);
    std::move(moving_close_spacetime_objects.begin(),
              moving_close_spacetime_objects.end(),
              std::back_inserter(close_spacetime_objects));
  }
  if (!close_spacetime_objects.empty()) {
    MakeCloseObjectSlowdownDecision(close_spacetime_objects,
                                    *input.drive_passage, *input.path,
                                    *input.path_sl_boundary, constraint_mgr);
  }
}

}  // namespace planner
}  // namespace qcraft
