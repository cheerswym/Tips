#ifndef ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
#define ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "onboard/async/thread_pool.h"
#include "onboard/lidar/vehicle_model/vehicle_model.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/teleop_state.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {

struct StPathPlannerInput {
  int plan_id = 0;
  const StPathPlanStartPointInfo *start_point_info = nullptr;
  const SchedulerOutput *scheduler_output = nullptr;
  const VehicleGeometryParamsProto *vehicle_geom_params = nullptr;
  const VehicleDriveParamsProto *vehicle_drive_params = nullptr;
  const PlannerParamsProto *planner_params = nullptr;
  const PlannerObjectManager *obj_mgr = nullptr;
  const PlannerSemanticMapManager *planner_semantic_map_manager = nullptr;
  const absl::flat_hash_set<std::string> *stalled_objects = nullptr;
  const SceneOutputProto *scene_reasoning = nullptr;
  const TrafficLightInfoMap *tl_info_map = nullptr;
  const PoseProto *pose = nullptr;
  const TeleopState *teleop_state = nullptr;
  const VehicleModel *vehicle_model = nullptr;
  const std::vector<ApolloTrajectoryPointProto> *time_aligned_prev_traj =
      nullptr;
  const SensorFovsProto *sensor_fovs = nullptr;
  const SpacetimePlannerTrajectories *prev_st_planner_trajectories = nullptr;
  const DeciderStateProto *prev_decider_state = nullptr;
  const InitializerStateProto *prev_initializer_state = nullptr;
  absl::Time parking_brake_release_time;
};

struct StPathPlannerOutput {
  DiscretizedPath path;
  std::unique_ptr<SpacetimeTrajectoryManager> traj_mgr;
  InitializerDebugProto initializer_debug_proto;
  vis::vantage::ChartDataBundleProto chart_data;
  TrajectoryOptimizerDebugProto optimizer_debug_proto;
  AccumulatedDiscountedCostsProto opt_feature_costs;
  AccumulatedDiscountedCostsProto opt_expert_costs;
  ConstraintManager constraint_manager;
  InitializerStateProto initializer_state;
  DeciderStateProto decider_state;
};

PlannerStatus RunStPathPlanner(const StPathPlannerInput &input,
                               StPathPlannerOutput *out,
                               ThreadPool *thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ST_PATH_PLANNER_H_
