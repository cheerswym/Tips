#ifndef ONBOARD_PLANNER_DECISION_DECIDER_INPUT_H_
#define ONBOARD_PLANNER_DECISION_DECIDER_INPUT_H_

#include <limits>
#include <string>

#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/tl_info.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {

struct DeciderInput {
  const qcraft::VehicleGeometryParamsProto *vehicle_geometry_params = nullptr;
  const DecisionConstraintConfigProto *config = nullptr;
  const PlannerSemanticMapManager *planner_semantic_map_manager = nullptr;
  const absl::flat_hash_set<std::string> *stalled_objects = nullptr;
  const SceneOutputProto *scene_reasoning = nullptr;
  const LaneChangeStage lc_stage;
  const ApolloTrajectoryPointProto *plan_start_point = nullptr;
  const mapping::LanePath *lane_path_before_lc = nullptr;
  const DrivePassage *passage = nullptr;
  const PathSlBoundary *sl_boundary = nullptr;
  bool borrow_lane_boundary = false;
  const PlannerObjectManager *obj_mgr = nullptr;
  const SpacetimeTrajectoryManager *st_traj_mgr = nullptr;
  const TrafficLightInfoMap *tl_info_map = nullptr;
  const std::optional<ClearanceCheckOutput> *lc_clearance_check_output =
      nullptr;
  const DeciderStateProto *pre_decider_state = nullptr;
  const PoseProto *pose = nullptr;
  const FrenetBox *av_frenet_box = nullptr;
  absl::Time parking_brake_release_time;
  bool teleop_enable_traffic_light_stop;
  bool enable_pull_over;
  double brake_to_stop = -1.0;
  double length_along_route = std::numeric_limits<double>::max();
  // TODO(PNC-501): This is a hack, remove later.
  VehicleModel vehicle_model;
  absl::Time plan_time;
  const SensorFovsProto *sensor_fovs = nullptr;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_DECIDER_INPUT_H_
