#ifndef ONBOARD_PLANNER_PLAN_FREE_DRIVE_TASK_H_
#define ONBOARD_PLANNER_PLAN_FREE_DRIVE_TASK_H_

#include "absl/status/statusor.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::planner {

struct FreeDriveTaskOutput {
  TrajectoryProto trajectory_info;
  FreespacePlannerDebugProto debug_proto;
  vis::vantage::ChartsDataProto chart_data;
};

struct FreeDriveTaskInput {
  bool reset;
  const PlannerSemanticMapManager* psmm;
  const CoordinateConverter* coordinate_converter;
  const GlobalPose* goal;
  const PoseProto* pose;
  const Chassis* chassis;
  const PlanStartPointInfo* plan_start_point_info;
  absl::Time plan_time;
  const PlannerParams* planner_params;
  const VehicleGeometryParamsProto* veh_geo_params;
  const VehicleDriveParamsProto* veh_drive_params;
  const TrajectoryProto* prev_trajectory_proto;
  const PlannerObjectManager* object_manager;
};

absl::Status RunFreeDriveTask(const FreeDriveTaskInput& input,
                              FreespacePlannerStateProto* state,
                              FreeDriveTaskOutput* result,
                              ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_FREE_DRIVE_TASK_H_
