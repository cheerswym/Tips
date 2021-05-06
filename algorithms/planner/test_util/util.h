#ifndef ONBOARD_PLANNER_TEST_UTIL_UTIL_H_
#define ONBOARD_PLANNER_TEST_UTIL_UTIL_H_

#include "onboard/math/vec.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

PoseProto CreatePose(double timestamp, Vec2d pos, double heading,
                     Vec2d speed_vec);

ApolloTrajectoryPointProto ConvertToTrajPointProto(const PoseProto &pose);

// Create a default vehicle geometry.
VehicleGeometryParamsProto DefaultVehicleGeometry();

// Create a default vehicle drive parameter without control calibration tables.
VehicleDriveParamsProto DefaultVehicleDriveParams();

PlannerParamsProto DefaultPlannerParams();

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_UTIL_H_
