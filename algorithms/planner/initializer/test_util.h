#ifndef ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "gmock/gmock.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft {
namespace planner {

// Match CollisionInfo result.
MATCHER_P2(ObjCollisionEq, index, time, "") {
  *result_listener << "Actual value: index: " << arg.object_index
                   << ", time: " << arg.time;
  return arg.object_index == index && arg.time == time;
}

// Build a geometry graph from a straight line. A node is created for each point
// in `points`. Edges are created between consecutive nodes.
XYGeometryGraph BuildLineGraph(
    absl::Span<const Vec2d> points,
    std::vector<std::unique_ptr<GeometryForm>>* ptr_geometry_forms);

// Build a motion graph with constant acceleration.
std::pair<std::unique_ptr<XYGeometryGraph>, std::unique_ptr<XYTMotionGraph>>
BuildConstAccelLineGraph(absl::Span<const Vec2d> points, double init_speed,
                         double init_time, double accel);

// Create a fake sl boundary from a drive passage with half width 1.5m.
PathSlBoundary CreateFakePathSlBoundary(const DrivePassage& passage);

};  // namespace planner

}  // namespace qcraft

#endif  // ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_
