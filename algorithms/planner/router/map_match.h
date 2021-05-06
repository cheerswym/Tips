#ifndef ONBOARD_PLANNER_ROUTER_MAP_MATCH_H_
#define ONBOARD_PLANNER_ROUTER_MAP_MATCH_H_

#include <vector>

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route::map_match {

inline Vec2d LookAheadPos(double x, double y, double speed, double angle_rad,
                          double secs);

absl::StatusOr<mapping::PointToLane> GetNearestLaneOnDriving(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const RouteParamProto &route_param_proto, std::int64_t cur_time_micros);

std::vector<mapping::PointToLane> GetNearLanesFromPose(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const RouteMapMatchParam &match_param, std::int64_t cur_time_micros);

}  // namespace qcraft::planner::route::map_match

#endif
