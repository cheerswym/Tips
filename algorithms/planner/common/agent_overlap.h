#ifndef ONBOARD_PLANNER_COMMON_POLYGON_OVERLAP_H_
#define ONBOARD_PLANNER_COMMON_POLYGON_OVERLAP_H_

#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/polygon2d_util.h"
#include "onboard/planner/common/path_approx.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// A single agent state's overlap.
struct AgentOverlap {
  double first_ra_s;
  double last_ra_s;
  // 0.0 if there is a true overlap. It is negative if the agent state is at the
  // right side of the path, and it is positive if the agent state is at the
  // left side of the overlap.
  double lat_dist;
};

// Compute the overlaps between one agent state and a path approximation in
// range [first_index, last_index]. `max_lat_dist` is the maximum lateral
// distance to compute overlap.
std::vector<AgentOverlap> ComputeAgentOverlaps(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    const VehicleGeometryParamsProto& vehicle_geom);

struct AgentNearestPoint {
  // Represents the rear axle s when the agent state is closest to the front of
  // the vehicle.
  double nearest_ra_s = 0.0;
  // It is negative if the agent state is at the right side of the path, and it
  // is positive if the agent state is at the left side of the path. 0.0 if the
  // nearest point is on the boundary of path_segment.
  double lat_dist = 0.0;
};

// Compute the nearest ra_s between one agent state and a path approximation in
// range [first_index, last_index]. `max_lat_dist` is the maximum lateral
// distance to compute nearest ra_s.
std::optional<AgentNearestPoint> ComputeAgentNearestPoint(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_POLYGON_OVERLAP_H_
