#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

struct SelectorInput {
  const mapping::SemanticMapManager *smm;
  const RouteSectionsInfo *sections_info;
  const std::vector<LanePathInfo> *lane_path_infos;
  const mapping::LanePath *prev_lane_path_from_current;
  const std::vector<ApolloTrajectoryPointProto> *prev_traj;
  const MotionConstraintParamsProto *motion_constraints;
  const VehicleGeometryParamsProto *vehicle_geom;
  const ApolloTrajectoryPointProto *plan_start_point;
  const absl::flat_hash_set<std::string> *stalled_objects;
  const PredictionDebugProto *prediction_debug;

  const SelectorParamsProto *config;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_
