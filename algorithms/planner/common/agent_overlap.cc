#include "onboard/planner/common/agent_overlap.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "onboard/math/geometry/polygon2d_util.h"
#include "onboard/planner/common/path_approx.h"

namespace qcraft {
namespace planner {

namespace {
bool IsConsideredOverlap(const polygon2d::PolygonBoxOverlap& overlap) {
  return overlap.lat_dist != 0.0 || overlap.in != overlap.out;
}

// Approximates the polygon as a circle and quickly check if we need to compute
// overlap.
bool MaybeOverlap(const PathSegment& segment, const Polygon2d& polygon,
                  double max_lat_dist) {
  if (Sqr(segment.radius() + polygon.CircleRadius() + max_lat_dist) <
      polygon.CircleCenter().DistanceSquareTo(segment.center())) {
    return false;
  }
  if (std::abs(segment.tangent().CrossProd(polygon.CircleCenter() -
                                           segment.center())) >
      max_lat_dist + segment.half_width() + polygon.CircleRadius()) {
    return false;
  }
  if (std::abs(
          segment.tangent().Dot(polygon.CircleCenter() - segment.center())) >
      max_lat_dist + segment.half_length() + polygon.CircleRadius()) {
    return false;
  }
  return true;
}

AgentOverlap ConvertToAgentOverlap(
    const PathApprox& path_approx, int segment_index, double av_length,
    const polygon2d::PolygonBoxOverlap& overlap) {
  const auto& segment = path_approx.segment(segment_index);
  double start_ra_s =
      std::clamp(segment.last_s() - (segment.length() - overlap.in),
                 segment.first_s(), segment.last_s());
  double last_ra_s =
      std::clamp(segment.first_s() + overlap.out, start_ra_s, segment.last_s());

  return AgentOverlap{.first_ra_s = start_ra_s,
                      .last_ra_s = last_ra_s,
                      .lat_dist = overlap.lat_dist};
}

AgentNearestPoint ConvertToAgentNearestPoint(
    const PathApprox& path_approx, int segment_index,
    const polygon2d::PolygonBoxOverlap& overlap) {
  QCHECK_EQ(overlap.in, overlap.out);
  const auto& segment = path_approx.segment(segment_index);
  const double closest_ra_s =
      std::clamp(segment.last_s() - (segment.length() - overlap.in),
                 segment.first_s(), segment.last_s());

  return AgentNearestPoint{.nearest_ra_s = closest_ra_s,
                           .lat_dist = overlap.lat_dist};
}

bool ClampAgentOverlap(double low, double high, AgentOverlap* overlap) {
  if (low > overlap->last_ra_s || high < overlap->first_ra_s) {
    return false;
  }
  overlap->first_ra_s = std::max(overlap->first_ra_s, low);
  overlap->last_ra_s = std::min(overlap->last_ra_s, high);
  return true;
}

}  // namespace

std::vector<AgentOverlap> ComputeAgentOverlaps(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    const VehicleGeometryParamsProto& vehicle_geom) {
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;
  const double av_length = vehicle_geom.length();

  std::vector<AgentOverlap> agent_overlaps;
  double min_abs_dist = std::numeric_limits<double>::max();

  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    if (!MaybeOverlap(path_approx.segment(i), polygon, max_lat_dist)) continue;

    const auto geom_overlap =
        polygon2d::ComputePolygonBoxOverlap(polygon, path_approx.segment(i));
    if (!IsConsideredOverlap(geom_overlap)) continue;

    // Skip if this is not the laterally nearest overlap.
    if (std::abs(geom_overlap.lat_dist) > min_abs_dist) continue;

    min_abs_dist = std::abs(geom_overlap.lat_dist);

    auto agent_overlap =
        ConvertToAgentOverlap(path_approx, i, av_length, geom_overlap);

    if (!ClampAgentOverlap(first_s, last_s, &agent_overlap)) continue;

    agent_overlaps.push_back(agent_overlap);
  }

  // Prune all overlaps that is larger than min_abs_dist.
  agent_overlaps.erase(
      std::remove_if(agent_overlaps.begin(), agent_overlaps.end(),
                     [min_abs_dist](const auto& o) {
                       return std::abs(o.lat_dist) > min_abs_dist;
                     }),
      agent_overlaps.end());

  return agent_overlaps;
}

std::optional<AgentNearestPoint> ComputeAgentNearestPoint(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist) {
  const int first_seg_index = path_approx.PointToSegmentIndex(first_index);
  const int last_seg_index = path_approx.PointToSegmentIndex(last_index);
  const double first_s = step_length * first_index;
  const double last_s = step_length * last_index;

  std::optional<AgentNearestPoint> agent_close;
  double min_abs_dist = max_lat_dist;
  for (int i = first_seg_index; i <= last_seg_index; ++i) {
    if (!MaybeOverlap(path_approx.segment(i), polygon, max_lat_dist)) continue;
    const auto geom_overlap =
        polygon2d::ComputePolygonBoxOverlap(polygon, path_approx.segment(i));
    if (geom_overlap.in != geom_overlap.out) continue;
    const double abs_dist = std::abs(geom_overlap.lat_dist);
    if (abs_dist < min_abs_dist) {
      auto curr_agent_close =
          ConvertToAgentNearestPoint(path_approx, i, geom_overlap);
      if (curr_agent_close.nearest_ra_s < first_s ||
          curr_agent_close.nearest_ra_s > last_s) {
        continue;
      }
      min_abs_dist = abs_dist;
      agent_close = std::move(curr_agent_close);
    }
  }
  return agent_close;
}

}  // namespace planner
}  // namespace qcraft
