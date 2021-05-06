#ifndef ONBOARD_PLANNER_DECISION_PRECEDENCE_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_PRECEDENCE_DECIDER_H_

#include "onboard/maps/lane_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/smooth_lane_path.h"
#include "onboard/prediction/prediction.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {
namespace planner {

PrecedenceLevel DecidePrecedenceForVehicle(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &av_path,
    const mapping::LanePath &agent_path,
    const prediction::PredictedTrajectory &agent_traj);

PrecedenceLevel DecidePrecedenceForLanePaths(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &av_path,
    const mapping::LanePath &agent_path);

PrecedenceLevel DecidePrecedenceForLanePaths(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &av_path,
    const mapping::LanePath &agent_path,
    const SmoothLanePath &av_smooth_lane_path,
    const SmoothLanePath &agent_smooth_lane_path);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_PRECEDENCE_DECIDER_H_
