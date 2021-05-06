#ifndef ONBOARD_PLANNER_DECISION_IGNORE_OBJECT_H_
#define ONBOARD_PLANNER_DECISION_IGNORE_OBJECT_H_

#include <string>
#include <vector>

#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

struct IgnoreObject {
  enum Reason {
    CANNOT_STOP_BEFORE_FOD = 1,
    CANNOT_STOP_BEFORE_VEGETATION = 2,
  };
  const SpacetimeObjectTrajectory* spacetime_object;
  std::string traj_id;
  Reason reason;
};

// This function return objects should be ignored. Currently, only consider
// object type OT_FOD and OT_VEGETATION. For more details see
// 'ignore_object.md'.
std::vector<ConstraintProto::IgnoreObjectProto> FindObjectsToIgnore(
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const DrivePassage& passage, const SpacetimeTrajectoryManager& st_traj_mgr,
    const PoseProto& av_pose);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_IGNORE_OBJECT_H_
