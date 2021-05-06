#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct FreespacePlannerOutput {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  Chassis::GearPosition gear_position;
  bool low_speed_freespace = false;
};

struct FreespacePlannerInput {
  bool new_task;
  const PoseProto *ego_pose;
  const CoordinateConverter *coordinate_converter;
  const Chassis *chassis;
  const VehicleGeometryParamsProto *veh_geo_params;
  const VehicleDriveParamsProto *veh_drive_params;
  const PlannerParams *planner_params;
  const PlannerObjectManager *obj_mgr;
  const PlannerSemanticMapManager *psmm;
  const absl::flat_hash_set<std::string> *stalled_object_ids;
  const ApolloTrajectoryPointProto *plan_start_point;
  bool start_point_reset;
  absl::Time plan_time;
  const FreespaceMap *freespace_map;
  // params
  const FreespaceParamsProto *freespace_param;
};

absl::StatusOr<FreespacePlannerOutput> RunFreespacePlanner(
    const FreespacePlannerInput &input, FreespacePlannerStateProto *state,
    FreespacePlannerDebugProto *debug_info,
    vis::vantage::ChartsDataProto *charts_data, ThreadPool *thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_H_
