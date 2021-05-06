#include "onboard/planner/object/drive_passage_filter.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "onboard/eval/qevent.h"
#include "onboard/math/util.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

namespace {

// Returns the object distance buffer in meters. If the object's distance to sl
// boundary is larger than this value, we may ignore the object.
double GetDistanceBuffer(const PlannerObject& object) {
  switch (object.type()) {
    case OT_UNKNOWN_STATIC:
    case OT_FOD:
    case OT_VEGETATION:
    case OT_BARRIER:
    case OT_CONE:
    case OT_WARNING_TRIANGLE:
      return 0.3;
    case OT_VEHICLE:
    case OT_UNKNOWN_MOVABLE:
    case OT_MOTORCYCLIST:
    case OT_PEDESTRIAN:
    case OT_CYCLIST:
      return object.is_stationary() ? 0.3 : 0.8;
  }
}

// Returns the right most lateral shift and left most lateral shift on the sl
// boundary from min_s to max_s.
std::pair<double, double> FindMaxRangeOfL(const PathSlBoundary& sl_boudnary,
                                          double min_s, double max_s) {
  static constexpr double kSampleStep = 0.5;
  double min_lat = -std::numeric_limits<double>::infinity();
  double max_lat = std::numeric_limits<double>::infinity();
  auto [r, l] = sl_boudnary.QueryBoundaryL(max_s);
  min_lat = Min(min_lat, r, l);
  max_lat = Max(max_lat, r, l);
  for (double s = min_s; s < max_s; s += kSampleStep) {
    const auto [r, l] = sl_boudnary.QueryBoundaryL(s);
    min_lat = Min(min_lat, r, l);
    max_lat = Max(max_lat, r, l);
  }
  return {min_lat, max_lat};
}

// Check if the polygon has lateral overlap.
bool HasLateralOverlap(const DrivePassage& drive_passage,
                       const Polygon2d& contour, double boundary_min_l,
                       double boundary_max_l) {
  QCHECK_GE(boundary_max_l, boundary_min_l);

  double contour_min_l = std::numeric_limits<double>::infinity();
  double contour_max_l = -std::numeric_limits<double>::infinity();
  bool has_projection = false;
  for (const auto& pt : contour.points()) {
    ASSIGN_OR_CONTINUE(const auto offset,
                       drive_passage.QueryFrenetLatOffsetAt(pt));
    has_projection = true;
    UpdateMin(offset, &contour_min_l);
    UpdateMax(offset, &contour_max_l);
  }
  if (!has_projection) return false;

  return contour_min_l <= boundary_max_l && contour_max_l >= boundary_min_l;
}

bool TrajectoryMaybeHasOverlap(const DrivePassage& drive_passage,
                               const PathSlBoundary& sl_boundary,
                               const prediction::PredictedTrajectory& traj,
                               double padding) {
  const auto& points = traj.points();
  for (int i = 0, n = points.size(); i < n; ++i) {
    const auto projection_or =
        drive_passage.QueryLaterallyUnboundedFrenetCoordinateAt(
            points[i].pos());
    if (!projection_or.ok()) {
      continue;
    }
    const auto& projection = *projection_or;
    // Estimate the lower bound distance to the sl boundary.
    double dist = 0.0;
    if (projection.s < drive_passage.front_s()) {
      dist = drive_passage.front_s() - projection.s;
    } else if (projection.s > drive_passage.end_s()) {
      dist = projection.s - drive_passage.end_s();
    } else {
      const auto [right_l, left_l] = sl_boundary.QueryBoundaryL(projection.s);
      if (projection.l < right_l) {
        dist = right_l - projection.l;
      } else if (projection.l > left_l) {
        dist = projection.l - left_l;
      } else {
        dist = 0.0;
      }
    }

    if (dist < padding) return true;

    // Walk along the trajectory until the distance is no more than padding.
    for (++i; i < n && dist > padding; ++i) {
      // HACK(lidong): This is used to prevent negative s. Delete this code once
      // negative s problem is not found in the following event.
      if (points[i - 1].s() > points[i].s() + 1e-3) {
        QEVENT_EVERY_N_SECONDS("lidong", "negative_s_in_prediction", 1.0,
                               [](QEvent* event) {});
      }
      dist -= std::abs(points[i].s() - points[i - 1].s());
    }
    --i;
  }
  return false;
}

}  // namespace

FilterReason::Type DrivePassageFilter::Filter(
    const PlannerObject& object,
    const prediction::PredictedTrajectory& traj) const {
  const auto& contour = object.contour();

  const double circle_radius = contour.CircleRadius();
  const auto obj_center = contour.CircleCenter();

  const double distance_buffer = GetDistanceBuffer(object);

  const double padding = circle_radius + distance_buffer;
  if (prediction::IsStationaryTrajectory(traj)) {
    // TODO(lidong): Consider combine the two queries into one.
    const auto projection_or =
        drive_passage_->QueryLaterallyUnboundedFrenetCoordinateAt(
            contour.CircleCenter());
    if (!projection_or.ok()) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_DRIVE_PASSAGE;
    }
    const auto& projection = *projection_or;
    if (projection.s < drive_passage_->front_s() - padding ||
        projection.s > drive_passage_->end_s() + padding) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_DRIVE_PASSAGE;
    }

    // Use curb to filter stationary objects.
    const auto station_index =
        drive_passage_->FindNearestStationIndex(obj_center);
    const auto& station = drive_passage_->station(station_index);
    const auto boundaries = station.boundaries();
    const double right_curb = boundaries.front().lat_offset;
    const double left_curb = boundaries.back().lat_offset;

    if (projection.l < right_curb - padding ||
        projection.l > left_curb + padding) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_DRIVE_PASSAGE;
    }

    // Use SL boundary to filter stationary objects.
    const auto [min_right_l, max_left_l] =
        FindMaxRangeOfL(*sl_boundary_, projection.s - circle_radius,
                        projection.s + circle_radius);
    QCHECK_LE(min_right_l, max_left_l);
    if (projection.l < min_right_l - padding ||
        projection.l > max_left_l + padding) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_SL_BOUNDARY;
    }

    if (!HasLateralOverlap(*drive_passage_, contour,
                           /*boundary_min_l=*/min_right_l - distance_buffer,
                           /*boundary_max_l=*/max_left_l + distance_buffer)) {
      return FilterReason::STATIONARY_OBJECT_NOT_ON_SL_BOUNDARY;
    }
    return FilterReason::NONE;
  }

  // Filtering moving objects.
  if (!TrajectoryMaybeHasOverlap(*drive_passage_, *sl_boundary_, traj,
                                 padding)) {
    return FilterReason::TRAJECTORY_NOT_ON_SL_BOUNDARY;
  }

  return FilterReason::NONE;
}
}  // namespace planner
}  // namespace qcraft
