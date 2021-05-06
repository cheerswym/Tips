#ifndef ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_
#define ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_

#include <vector>

#include "onboard/planner/common/lane_path_info.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

std::vector<LanePathInfo> FilterMultipleTargetLanePath(
    const RouteSectionsInfo &route_sections_info,
    const mapping::LanePath &last_target_lane_path,
    const ApolloTrajectoryPointProto &plan_start_point,
    const mapping::LanePath &preferred_lane_path,
    std::vector<LanePathInfo> *mutable_lp_infos);

LanePathInfo ChooseLeastLateralOffsetLanePath(
    const std::vector<LanePathInfo> &lp_infos,
    const std::vector<ApolloTrajectoryPointProto> &trajectory);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SCHEDULER_TARGET_LANE_PATH_FILTER_H_
