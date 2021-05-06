#ifndef ONBOARD_PLANNER_STANDBY_STATE_H_
#define ONBOARD_PLANNER_STANDBY_STATE_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
absl::Status EnterStandbyState(const PoseProto &pose,
                               const VehicleGeometryParamsProto &vehicle_geom,
                               TrajectoryProto *trajectory,
                               PlannerDebugProto *planner_debug);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_STANDBY_STATE_H_
