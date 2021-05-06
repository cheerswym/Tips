#include "onboard/planner/speed/path_semantic_analyzer.h"

#include <algorithm>
#include <limits>

#include "onboard/async/parallel_for.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

inline LaneSemantic QueryLaneSemantic(mapping::ElementId lane_id,
                                      const PlannerSemanticMapManager& psmm) {
  const auto& lane_info = psmm.FindLaneInfoOrDie(lane_id);
  if (lane_info.is_in_intersection) {
    switch (lane_info.direction) {
      case mapping::LaneProto::STRAIGHT:
        return LaneSemantic::INTERSECTION_STRAIGHT;
      case mapping::LaneProto::LEFT_TURN:
        return LaneSemantic::INTERSECTION_LEFT_TURN;
      case mapping::LaneProto::RIGHT_TURN:
        return LaneSemantic::INTERSECTION_RIGHT_TURN;
      case mapping::LaneProto::UTURN:
        return LaneSemantic::INTERSECTION_UTURN;
    }
  } else {
    switch (lane_info.direction) {
      case mapping::LaneProto::STRAIGHT:
        return LaneSemantic::ROAD;
      case mapping::LaneProto::LEFT_TURN:
      case mapping::LaneProto::RIGHT_TURN:
      case mapping::LaneProto::UTURN:
        QLOG_EVERY_N(ERROR, 100)
            << "Lane " << lane_id << " direction is "
            << mapping::LaneProto::Direction_Name(lane_info.direction)
            << " but not in intersection.";
        return LaneSemantic::ROAD;
    }
  }
}

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    const DiscretizedPath& path, int max_analyze_path_index,
    const RouteSections& route_sections_within_dp,
    const PlannerSemanticMapManager& psmm, ThreadPool* thread_pool) {
  SCOPED_QTRACE("AnalyzePathSemantics");

  QCHECK(!path.empty());
  QCHECK_LT(max_analyze_path_index, path.size());

  const int path_semantic_size = max_analyze_path_index + 1;
  std::vector<PathPointSemantic> path_semantics(path_semantic_size);

  ASSIGN_OR_RETURN(
      const auto lane_paths_in_route_sections,
      CollectAllLanePathOnRouteSections(psmm, route_sections_within_dp));
  ParallelFor(0, path_semantic_size, thread_pool, [&](int i) {
    // TODO(renjie): Consider to use lane boundary and AV box info to tell which
    // lane the path point is located in exactly.
    // TODO(renjie): Optimize the performance.
    const auto& path_point = path[i];
    const Vec2d path_point_xy = ToVec2d(path_point);
    double min_dist_sqr = std::numeric_limits<double>::infinity();
    mapping::LanePoint closest_lane_point(
        /*lane_id=*/mapping::kInvalidElementId,
        /*fraction=*/0.0);
    Vec2d closest_lane_point_pos;
    for (const auto& lane_path : lane_paths_in_route_sections) {
      Vec2d closest_point;
      ASSIGN_OR_CONTINUE(
          const auto lane_point,
          FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
              psmm.semantic_map_manager()->GetLevel(),
              *psmm.semantic_map_manager(), path_point_xy, lane_path,
              path_point.theta(), /*heading_penalty_weight=*/0.0,
              &closest_point));
      const double dist_sqr = path_point_xy.DistanceSquareTo(closest_point);
      if (dist_sqr < min_dist_sqr) {
        min_dist_sqr = dist_sqr;
        closest_lane_point = lane_point;
        closest_lane_point_pos = closest_point;
      }
    }
    if (!closest_lane_point.Valid()) {
      return;
    }
    path_semantics[i].closest_lane_point = closest_lane_point;
    path_semantics[i].closest_lane_point_pos = closest_lane_point_pos;
    path_semantics[i].lane_semantic =
        QueryLaneSemantic(path_semantics[i].closest_lane_point.lane_id(), psmm);
    return;
  });

  for (int i = 0; i < path_semantic_size; ++i) {
    if (!path_semantics[i].closest_lane_point.Valid()) {
      return absl::NotFoundError(
          absl::StrCat("Failed to find closest lane point for path point[", i,
                       "]: ", path[i].DebugString()));
    }
  }

  // Set lane path id. If two adjacent closest lane points have a larger squared
  // distance than this, we consider that a lane change happens.
  constexpr double kClosestLanePointSqrDistThres = Sqr(2.8);  // m^2.
  std::vector<int> curr_lane_path_id_history = {0};
  path_semantics[0].lane_path_id_history = curr_lane_path_id_history;
  for (int i = 1; i < path_semantic_size; ++i) {
    if (path_semantics[i].closest_lane_point.lane_id() !=
            path_semantics[i - 1].closest_lane_point.lane_id() &&
        path_semantics[i].closest_lane_point_pos.DistanceSquareTo(
            path_semantics[i - 1].closest_lane_point_pos) >
            kClosestLanePointSqrDistThres) {
      const auto closest_lane_point_tangent =
          path_semantics[i].closest_lane_point.ComputeTangent(
              *psmm.semantic_map_manager());
      if (closest_lane_point_tangent.CrossProd(
              path_semantics[i].closest_lane_point_pos -
              path_semantics[i - 1].closest_lane_point_pos) > 0.0) {
        // Lane change left.
        curr_lane_path_id_history.push_back(curr_lane_path_id_history.back() +
                                            1);
      } else {
        // Lane change right.
        curr_lane_path_id_history.push_back(curr_lane_path_id_history.back() -
                                            1);
      }
    }
    path_semantics[i].lane_path_id_history = curr_lane_path_id_history;
  }

  return path_semantics;
}

}  // namespace planner
}  // namespace qcraft
