#ifndef ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
#define ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/async/thread_pool.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"

namespace qcraft {
namespace planner {

enum class LaneSemantic {
  NONE = 0,
  ROAD = 1,
  INTERSECTION_LEFT_TURN = 2,
  INTERSECTION_RIGHT_TURN = 3,
  INTERSECTION_STRAIGHT = 4,
  INTERSECTION_UTURN = 5
};

struct PathPointSemantic {
  mapping::LanePoint closest_lane_point;
  Vec2d closest_lane_point_pos;
  LaneSemantic lane_semantic = LaneSemantic::NONE;
  // Keep the lane path id history from path beginning to the current point.
  // Lane path id starts from zero, and is increased by one if there is a left
  // lane change and reduced by one if there is a right lane change.
  // Examples:
  // 1) { 0 } means there is no lane change from path beginning to
  // current point;
  // 2) { 0, 1 } means there is a left lane change from path
  // beginning to current point;
  // 3) { 0, 1, 2 } means there are two
  // succesive left lane changes from path beginning to current point;
  // 4) { 0, 1, 0 } means current point returns to the original lane after a
  // left and a right lane change.
  std::vector<int> lane_path_id_history;
};

absl::StatusOr<std::vector<PathPointSemantic>> AnalyzePathSemantics(
    const DiscretizedPath& path, int max_analyze_path_index,
    const RouteSections& route_sections_within_dp,
    const PlannerSemanticMapManager& psmm, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_PATH_SEMANTIC_ANALYZER_H_
