#ifndef ONBOARD_PLANNER_INITIALIZER_INITIALIZER_INPUT_H_
#define ONBOARD_PLANNER_INITIALIZER_INITIALIZER_INPUT_H_

#include <memory>
#include <string>
#include <vector>

#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

struct MotionSearchInput {
  const ApolloTrajectoryPointProto *start_point = nullptr;
  absl::Time plan_time;
  const PoseProto *sdc_pose = nullptr;
  const DrivePassage *drive_passage = nullptr;
  const SpacetimeTrajectoryManager *st_traj_mgr = nullptr;
  const VehicleGeometryParamsProto *vehicle_geom = nullptr;
  const GeometryGraph *geom_graph = nullptr;
  const GeometryFormBuilder *form_builder = nullptr;
  const CollisionChecker *collision_checker = nullptr;
  const ConstraintManager *constraint_manager = nullptr;
  const PathSlBoundary *sl_boundary = nullptr;
  const PlannerParamsProto *planner_params = nullptr;
  const ConstraintProto::LeadingObjectProto *blocking_static_obj = nullptr;
  const std::optional<ClearanceCheckOutput> *lc_clearance = nullptr;
  const std::vector<InitializerSearchConfig> *search_configs = nullptr;
  bool is_lane_change = false;
  bool lc_multiple_traj = false;
};

struct InitializerInput {
  const StPathPlanStartPointInfo *start_point_info = nullptr;
  const PoseProto *sdc_pose = nullptr;
  const LaneChangeStateProto *lane_change_state = nullptr;
  bool lc_multiple_traj = false;
  const std::optional<ClearanceCheckOutput> *lc_clearance = nullptr;
  const absl::flat_hash_set<std::string> *stalled_objects = nullptr;
  const DrivePassage *drive_passage = nullptr;
  const SpacetimeTrajectoryManager *st_traj_mgr = nullptr;
  const VehicleGeometryParamsProto *vehicle_geom = nullptr;
  const VehicleDriveParamsProto *vehicle_drive = nullptr;
  const ConstraintManager *constraint_manager = nullptr;
  const PathSlBoundary *sl_boundary = nullptr;
  const InitializerStateProto *prev_initializer_state = nullptr;
  const PlannerParamsProto *planner_params = nullptr;
  int plan_id = 0;
};

struct ReferenceLineSearcherInput {
  const GeometryGraph *geometry_graph = nullptr;
  const DrivePassage *drive_passage = nullptr;
  const PathSlBoundary *sl_boundary = nullptr;
  const SpacetimeTrajectoryManager *st_traj_mgr = nullptr;
  const PlannerParamsProto *planner_params = nullptr;
  const VehicleGeometryParamsProto *vehicle_geom = nullptr;
  const VehicleDriveParamsProto *vehicle_drive = nullptr;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_INITIALIZER_INPUT_H_
