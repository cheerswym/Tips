#include "onboard/planner/router/map_match.h"

#include <cmath>

#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft::planner::route::map_match {

Vec2d LookAheadPos(double x, double y, double speed, double angle_rad,
                   double secs) {
  return {x + speed * secs * std::cos(angle_rad), y};
}

absl::StatusOr<mapping::PointToLane> GetNearestLaneOnDriving(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const RouteParamProto &route_param_proto, std::int64_t cur_time_micros) {
  auto points = GetNearLanesFromPose(semantic_map_manager, pose,
                                     route_param_proto.on_driving_param(),
                                     cur_time_micros);
  if (points.empty()) {
    return absl::NotFoundError(
        absl::StrCat("Cannot find lane when driving, ",
                     PoseDebugString(semantic_map_manager, pose)));
  }
  return points.front();
}

std::vector<mapping::PointToLane> GetNearLanesFromPose(
    const SemanticMapManager &semantic_map_manager, const PoseProto &pose,
    const RouteMapMatchParam &match_param, std::int64_t cur_time_micros) {
  const Vec2d pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  // TODO(xiang): check ahead pos, Now not enable,because failed to match road.
  //  double secs = (cur_time_micros - pose.header().timestamp()) / 1E6;
  //  pos = LookAheadPos(pose.pos_smooth().x(), pose.pos_smooth().x(),
  //                            pose.vel_body().x(), pose.yaw(), secs);
  const auto level = semantic_map_manager.GetLevel();
  return mapping::PointToNearLanesWithHeadingAtLevel(
      level, semantic_map_manager, pos, match_param.filter().radius_error(),
      pose.yaw(), match_param.filter().heading_error());
}

}  // namespace qcraft::planner::route::map_match
