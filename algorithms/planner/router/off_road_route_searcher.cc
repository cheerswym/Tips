#include "onboard/planner/router/off_road_route_searcher.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "onboard/math/geometry/line_intersection.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/route_manager_util.h"

namespace qcraft::planner {

std::vector<mapping::LanePoint> FindCrossingLanePointsFromPose(
    const SemanticMapManager &smm, const PoseProto &pose, double radius) {
  const Vec2d pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  const auto &lanes_info = smm.GetLanesInfoWithHeadingAtLevel(
      smm.GetLevel(), pos, pose.yaw(), radius,
      /*max_heading_diff=*/std::numeric_limits<double>::infinity());

  constexpr double kDefaultSearchAngle = M_PI_4;

  Polyline2d line_0(
      {pos, pos + Vec2d::FastUnitFromAngle(pose.yaw() - kDefaultSearchAngle) *
                      radius});
  Polyline2d line_1(
      {pos, pos + Vec2d::FastUnitFromAngle(pose.yaw() + kDefaultSearchAngle) *
                      radius});

  std::vector<mapping::LanePoint> res;
  for (const auto *lane_info : lanes_info) {
    Polyline2d lane_polyline(lane_info->points_smooth);

    double larger_arc_len = std::numeric_limits<double>::lowest();

    for (const auto &polyline : {line_0, line_1}) {
      Vec2d inter_point;
      double arc_len1, arc_len2;
      if (FindFirstIntersectionBetweenCurves(
              polyline, lane_polyline, &inter_point, &arc_len1, &arc_len2)) {
        larger_arc_len = std::max(larger_arc_len, arc_len2);
      }

      // TODO(weijun): Add boundary check.
    }

    if (larger_arc_len != std::numeric_limits<double>::lowest()) {
      res.emplace_back(lane_info->id, larger_arc_len / lane_info->length());
    }
  }

  return res;
}

absl::StatusOr<OffRoadRouteSearcherOutput> SearchForRouteFromOffRoad(
    const SemanticMapManager &smm, const PoseProto &pose,
    const MultipleStopsRequest &route_request, double search_radius) {
  const auto starting_lane_points =
      FindCrossingLanePointsFromPose(smm, pose, search_radius);

  if (starting_lane_points.empty()) {
    return absl::NotFoundError(
        "No near lane points are found within search radius.");
  }

  OffRoadRouteSearcherOutput output;

  double shortest_route_len = std::numeric_limits<double>::infinity();
  for (const auto &start_lane_point : starting_lane_points) {
    // TODO(weijun): Refactor FindNextDestinationIndexViaLanePoints to avoid
    // repeated computation.
    ASSIGN_OR_CONTINUE(
        const auto next_index,
        FindNextDestinationIndexViaLanePoints(
            smm, route_request,
            [&start_lane_point](const RouteSectionSequence &sections) {
              return sections.IsLanePointOnSections(start_lane_point);
            }));

    ASSIGN_OR_CONTINUE(
        const auto routing_request,
        route_request.GenerateRoutingRequestProtoToNextStop(smm, next_index));

    ASSIGN_OR_CONTINUE(auto route_result,
                       SearchForRouteLanePathFromLanePoint(
                           smm, start_lane_point, routing_request,
                           /*black_list=*/{}, /*use_time=*/true));

    const double route_len = route_result.first.length();
    if (route_len < shortest_route_len) {
      shortest_route_len = route_len;
      output.link_lane_point = start_lane_point;
      output.route_section_sequence = std::move(route_result.first);
      output.on_road_route_lane_path = std::move(route_result.second);
      output.next_stop_index = next_index;
    }
  }

  if (shortest_route_len == std::numeric_limits<double>::infinity()) {
    return absl::NotFoundError("No route found.");
  }

  return output;
}

}  // namespace qcraft::planner
