/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

// NOTE: This file is copied from Apollo project and modified by qcraft.ai for
// its own use.

#include "onboard/math/geometry/polygon2d.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "glog/logging.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace {

constexpr double kEpsilon = 1e-10;

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2) {
  return Vec2d(end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

AABox2d SegmentToAABox(const Segment2d &seg) {
  return AABox2d(seg.center(), seg.max_x() - seg.min_x(),
                 seg.max_y() - seg.min_y());
}

}  // namespace

Polygon2d::Polygon2d(const Box2d &box) : is_convex_(true) {
  points_ = box.GetCornersCounterClockwise();
  BuildFromPoints();
}

Polygon2d::Polygon2d(std::vector<Vec2d> points) : points_(std::move(points)) {
  BuildFromPoints();
  is_convex_ = AreConvexHullPoints(points_);
}

Polygon2d::Polygon2d(std::vector<Vec2d> points, bool is_convex)
    : points_(std::move(points)), is_convex_(is_convex) {
  BuildFromPoints();
  // Check convexity.
  if (is_convex_) {
    DCHECK(AreConvexHullPoints(points_));
  }
}

bool Polygon2d::IsSelfIntersecting() const {
  const int n = line_segments_.size();
  for (int i = 0; i < n; ++i) {
    const auto &seg_i = line_segments_[i];
    for (int j = i + 2; j < n - 1; ++j) {
      if (seg_i.HasIntersect(line_segments_[j])) {
        return true;
      }
    }
  }
  return false;
}

double Polygon2d::DistanceTo(const Vec2d &point) const {
  QCHECK_GE(points_.size(), 3);
  if (IsPointIn(point)) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segments_[i].DistanceTo(point));
  }
  return distance;
}

double Polygon2d::DistanceSquareTo(const Vec2d &point) const {
  QCHECK_GE(points_.size(), 3);
  if (IsPointIn(point)) {
    return 0.0;
  }
  double distance_sqr = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance_sqr =
        std::min(distance_sqr, line_segments_[i].DistanceSquareTo(point));
  }
  return distance_sqr;
}

double Polygon2d::DistanceTo(const Segment2d &line_segment) const {
  if (line_segment.length() <= kEpsilon) {
    return DistanceTo(line_segment.start());
  }
  QCHECK_GE(points_.size(), 3);
  if (IsPointIn(line_segment.center())) {
    return 0.0;
  }
  if (std::any_of(line_segments_.begin(), line_segments_.end(),
                  [&](const Segment2d &poly_seg) {
                    return poly_seg.HasIntersect(line_segment);
                  })) {
    return 0.0;
  }

  double distance = std::min(DistanceTo(line_segment.start()),
                             DistanceTo(line_segment.end()));
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segment.DistanceTo(points_[i]));
  }
  return distance;
}

double Polygon2d::DistanceTo(const Box2d &box) const {
  QCHECK_GE(points_.size(), 3);
  return DistanceTo(Polygon2d(box));
}

double Polygon2d::DistanceTo(const Polygon2d &polygon) const {
  QCHECK_GE(points_.size(), 3);
  QCHECK_GE(polygon.num_points(), 3);

  if (IsPointIn(polygon.points()[0])) {
    return 0.0;
  }
  if (polygon.IsPointIn(points_[0])) {
    return 0.0;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, polygon.DistanceTo(line_segments_[i]));
  }
  return distance;
}

double Polygon2d::DistanceToBoundary(const Vec2d &point) const {
  double distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    distance = std::min(distance, line_segments_[i].DistanceTo(point));
  }
  return distance;
}

bool Polygon2d::HasOverlap(const Box2d &box) const {
  return HasOverlap(Polygon2d(box));
}

bool Polygon2d::HasOverlap(const Polygon2d &polygon) const {
  QCHECK_GE(points_.size(), 3);
  if (polygon.max_x() < min_x() || polygon.min_x() > max_x() ||
      polygon.max_y() < min_y() || polygon.min_y() > max_y()) {
    return false;
  }

  if (polygon.IsPointIn(GetAllVertices()[0])) {
    return true;
  }

  for (const auto &segment : polygon.line_segments()) {
    if (HasOverlap(segment)) {
      return true;
    }
  }
  return false;
}

bool Polygon2d::Contains(const Segment2d &line_segment) const {
  if (line_segment.length() <= kEpsilon) {
    return IsPointIn(line_segment.start());
  }
  QCHECK_GE(points_.size(), 3);
  if (!IsPointIn(line_segment.start())) {
    return false;
  }
  if (!IsPointIn(line_segment.end())) {
    return false;
  }
  if (!is_convex_) {
    std::vector<Segment2d> overlaps = GetAllOverlaps(line_segment);
    double total_length = 0;
    for (const auto &overlap_seg : overlaps) {
      total_length += overlap_seg.length();
    }
    return total_length >= line_segment.length() - kEpsilon;
  }
  return true;
}

bool Polygon2d::Contains(const Polygon2d &polygon) const {
  QCHECK_GE(points_.size(), 3);
  if (area_ < polygon.area() - kEpsilon) {
    return false;
  }
  if (!IsPointIn(polygon.points()[0])) {
    return false;
  }
  const auto &line_segments = polygon.line_segments();
  return std::all_of(
      line_segments.begin(), line_segments.end(),
      [&](const Segment2d &line_segment) { return Contains(line_segment); });
}

int Polygon2d::Next(int at) const { return at >= num_points_ - 1 ? 0 : at + 1; }

int Polygon2d::Prev(int at) const { return at == 0 ? num_points_ - 1 : at - 1; }

void Polygon2d::BuildFromPoints() {
  num_points_ = static_cast<int>(points_.size());
  QCHECK_GE(num_points_, 3);

  // Make sure the points are in ccw order.
  area_ = 0.0;
  for (int i = 1; i < num_points_; ++i) {
    area_ += CrossProd(points_[0], points_[i - 1], points_[i]);
  }
  if (area_ < 0) {
    area_ = -area_;
    std::reverse(points_.begin(), points_.end());
  }
  area_ /= 2.0;

  // Construct line_segments.
  line_segments_.reserve(num_points_);
  for (int i = 0; i < num_points_; ++i) {
    line_segments_.emplace_back(points_[i], points_[Next(i)]);
  }

  aabox_ = AABox2d(points_);
}

bool Polygon2d::AreConvexHullPoints(absl::Span<const Vec2d> points) {
  const int n = points.size();
  if (n < 3) return false;
  const auto next = [n](int x) { return (x + 1) >= n ? (x + 1 - n) : (x + 1); };
  // For each edge, check if all the other points are all at the left side.
  for (int i = 0; i + 1 < n; ++i) {
    const Vec2d e = points[i + 1] - points[i];
    for (int j = (i + 2) % n; j != i; j = next(j)) {
      if (e.CrossProd(points[j] - points[i]) < kEpsilon) {
        return false;
      }
    }
  }
  return true;
}

bool Polygon2d::ComputeConvexHullPoints(
    absl::Span<const Vec2d> points, std::vector<Vec2d> *convex_hull_points) {
  QCHECK_NOTNULL(convex_hull_points)->clear();
  const int n = static_cast<int>(points.size());
  if (n < 3) {
    return false;
  }
  std::vector<int> sorted_indices(n);
  for (int i = 0; i < n; ++i) {
    sorted_indices[i] = i;
  }
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&](const int idx1, const int idx2) {
              const Vec2d &pt1 = points[idx1];
              const Vec2d &pt2 = points[idx2];
              const double dx = pt1.x() - pt2.x();
              if (std::abs(dx) > kEpsilon) {
                return dx < 0.0;
              }
              return pt1.y() < pt2.y();
            });
  int count = 0;
  std::vector<int> results;
  results.reserve(n);
  int last_count = 1;
  for (int i = 0; i < n + n; ++i) {
    if (i == n) {
      last_count = count;
    }
    const int idx = sorted_indices[(i < n) ? i : (n + n - 1 - i)];
    const Vec2d &pt = points[idx];
    while (count > last_count &&
           CrossProd(points[results[count - 2]], points[results[count - 1]],
                     pt) <= kEpsilon) {
      results.pop_back();
      --count;
    }
    results.push_back(idx);
    ++count;
  }
  --count;
  if (count < 3) {
    return false;
  }
  convex_hull_points->reserve(count);
  // Note(zheng): If the polygon edge length is too small, it
  // will return false when we use AreConvexHullPoints()
  // function, and also the segment of the polygon may be zero,
  // so we should remove the duplicated polygon points.
  convex_hull_points->push_back(points[results[0]]);
  int last_valid_index = 0;
  for (int i = 1; i < count; ++i) {
    // Remove duplicated polygon points.
    constexpr double kMaxDuplicatedDistSqr = 1e-20;  // m^2
    if (points[results[i]].ApproximatelyEquals(
            points[results[last_valid_index]], kMaxDuplicatedDistSqr)) {
      continue;
    }

    convex_hull_points->push_back(points[results[i]]);
    last_valid_index = i;
  }
  if (convex_hull_points->size() < 3) {
    convex_hull_points->clear();
    return false;
  }
  return true;
}

bool Polygon2d::ComputeConvexHull(absl::Span<const Vec2d> points,
                                  Polygon2d *const polygon) {
  QCHECK_NOTNULL(polygon);
  std::vector<Vec2d> result_points;
  ComputeConvexHullPoints(points, &result_points);
  if (result_points.size() < 3) return false;
  *polygon = Polygon2d(result_points, /*is_convex=*/true);
  return true;
}

Polygon2d Polygon2d::MergeTwoBoxes(const Box2d &first_box,
                                   const Box2d &second_box) {
  Polygon2d merged_polygon;
  std::vector<Vec2d> corners = first_box.GetCornersCounterClockwise();
  const std::vector<Vec2d> &second_corners =
      second_box.GetCornersCounterClockwise();
  corners.insert(corners.end(), second_corners.begin(), second_corners.end());
  Polygon2d::ComputeConvexHull(corners, &merged_polygon);
  return merged_polygon;
}

Polygon2d Polygon2d::MergeBoxes(const std::vector<Box2d> &boxes) {
  Polygon2d merged_polygon;
  std::vector<Vec2d> corners;
  for (const auto &box : boxes) {
    const std::vector<Vec2d> &current_corners =
        box.GetCornersCounterClockwise();
    corners.insert(corners.end(), current_corners.begin(),
                   current_corners.end());
  }
  Polygon2d::ComputeConvexHull(corners, &merged_polygon);
  return merged_polygon;
}

Polygon2d Polygon2d::MergeTwoPolygons(const Polygon2d &first_polygon,
                                      const Polygon2d &second_polygon) {
  Polygon2d merged_polygon;
  std::vector<Vec2d> corners = first_polygon.points();
  const std::vector<Vec2d> &second_corners = second_polygon.points();
  corners.insert(corners.end(), second_corners.begin(), second_corners.end());
  Polygon2d::ComputeConvexHull(corners, &merged_polygon);
  return merged_polygon;
}

bool Polygon2d::ClipConvexHull(const Segment2d &line_segment,
                               std::vector<Vec2d> *const points) {
  if (line_segment.length() <= kEpsilon) {
    return true;
  }
  QCHECK_NOTNULL(points);
  const size_t n = points->size();
  if (n < 3) {
    return false;
  }
  std::vector<double> prod(n);
  std::vector<int> side(n);
  for (size_t i = 0; i < n; ++i) {
    prod[i] = CrossProd(line_segment.start(), line_segment.end(), (*points)[i]);
    if (std::abs(prod[i]) <= kEpsilon) {
      side[i] = 0;
    } else {
      side[i] = ((prod[i] < 0) ? -1 : 1);
    }
  }

  std::vector<Vec2d> new_points;
  for (size_t i = 0; i < n; ++i) {
    if (side[i] >= 0) {
      new_points.push_back((*points)[i]);
    }
    const size_t j = ((i == n - 1) ? 0 : (i + 1));
    if (side[i] * side[j] < 0) {
      const double ratio = prod[j] / (prod[j] - prod[i]);
      new_points.emplace_back(
          (*points)[i].x() * ratio + (*points)[j].x() * (1.0 - ratio),
          (*points)[i].y() * ratio + (*points)[j].y() * (1.0 - ratio));
    }
  }

  points->swap(new_points);
  return points->size() >= 3;
}

bool Polygon2d::ComputeOverlap(const Polygon2d &other_polygon,
                               Polygon2d *const overlap_polygon) const {
  QCHECK_GE(points_.size(), 3);
  QCHECK_NOTNULL(overlap_polygon);
  QCHECK(is_convex_ && other_polygon.is_convex())
      << "Polygon2d this_polygon(" << DebugStringFullPrecision()
      << ");\n Polygon2d other_polygon(" << DebugStringFullPrecision() << ");";
  std::vector<Vec2d> points = other_polygon.points();
  for (int i = 0; i < num_points_; ++i) {
    if (!ClipConvexHull(line_segments_[i], &points)) {
      return false;
    }
  }
  return ComputeConvexHull(points, overlap_polygon) &&
         overlap_polygon->area_ > kEpsilon;
}

bool Polygon2d::HasOverlap(const Segment2d &line_segment) const {
  QCHECK_GE(points_.size(), 3);
  if (const auto aabox = SegmentToAABox(line_segment);
      !aabox.HasOverlap(aabox_)) {
    return false;
  }

  Vec2d first;
  Vec2d last;
  return GetOverlap(line_segment, &first, &last);
}

bool Polygon2d::GetOverlap(const Segment2d &line_segment, Vec2d *const first,
                           Vec2d *const last) const {
  QCHECK_GE(points_.size(), 3);
  QCHECK_NOTNULL(first);
  QCHECK_NOTNULL(last);

  if (line_segment.length() <= kEpsilon) {
    if (!IsPointIn(line_segment.start())) {
      return false;
    }
    *first = line_segment.start();
    *last = line_segment.start();
    return true;
  }

  double min_proj = line_segment.length();
  double max_proj = 0;
  if (IsPointIn(line_segment.start())) {
    *first = line_segment.start();
    min_proj = 0.0;
  }
  if (IsPointIn(line_segment.end())) {
    *last = line_segment.end();
    max_proj = line_segment.length();
  }
  for (const auto &poly_seg : line_segments_) {
    Vec2d pt;
    if (poly_seg.GetIntersect(line_segment, &pt)) {
      const double proj = line_segment.ProjectOntoUnit(pt);
      if (proj < min_proj) {
        min_proj = proj;
        *first = pt;
      }
      if (proj > max_proj) {
        max_proj = proj;
        *last = pt;
      }
    }
  }
  return min_proj <= max_proj + kEpsilon;
}

std::vector<Segment2d> Polygon2d::GetAllOverlaps(
    const Segment2d &line_segment) const {
  QCHECK_GE(points_.size(), 3);

  if (line_segment.length() <= kEpsilon) {
    std::vector<Segment2d> overlaps;
    if (IsPointIn(line_segment.start())) {
      overlaps.push_back(line_segment);
    }
    return overlaps;
  }
  std::vector<double> projections;
  if (IsPointIn(line_segment.start())) {
    projections.push_back(0.0);
  }
  if (IsPointIn(line_segment.end())) {
    projections.push_back(line_segment.length());
  }
  for (const auto &poly_seg : line_segments_) {
    Vec2d pt;
    if (poly_seg.GetIntersect(line_segment, &pt)) {
      projections.push_back(line_segment.ProjectOntoUnit(pt));
    }
  }
  std::sort(projections.begin(), projections.end());
  std::vector<std::pair<double, double>> overlaps;
  for (size_t i = 0; i + 1 < projections.size(); ++i) {
    const double start_proj = projections[i];
    const double end_proj = projections[i + 1];
    if (end_proj - start_proj <= kEpsilon) {
      continue;
    }
    const Vec2d reference_point =
        line_segment.start() +
        (start_proj + end_proj) / 2.0 * line_segment.unit_direction();
    if (!IsPointIn(reference_point)) {
      continue;
    }
    if (overlaps.empty() || start_proj > overlaps.back().second + kEpsilon) {
      overlaps.emplace_back(start_proj, end_proj);
    } else {
      overlaps.back().second = end_proj;
    }
  }
  std::vector<Segment2d> overlap_line_segments;
  for (const auto &overlap : overlaps) {
    overlap_line_segments.emplace_back(
        line_segment.start() + overlap.first * line_segment.unit_direction(),
        line_segment.start() + overlap.second * line_segment.unit_direction());
  }
  return overlap_line_segments;
}

void Polygon2d::ExtremePoints(const double heading, Vec2d *const first,
                              Vec2d *const last) const {
  ExtremePoints(Vec2d::UnitFromAngle(heading), first, last);
}

void Polygon2d::ExtremePoints(Vec2d direction_vec, Vec2d *first,
                              Vec2d *last) const {
  QCHECK(first != nullptr);
  QCHECK(last != nullptr);
  int first_index, last_index;
  ExtremePoints(direction_vec, &first_index, &last_index);
  *first = points()[first_index];
  *last = points()[last_index];
}

void Polygon2d::ExtremePoints(Vec2d direction_vec, int *first_idx,
                              int *last_idx, Vec2d *first_pt,
                              Vec2d *last_pt) const {
  DCHECK(first_idx != nullptr);
  DCHECK(last_idx != nullptr);
  DCHECK(first_pt != nullptr);
  DCHECK(first_pt != nullptr);
  ExtremePoints(direction_vec, first_idx, last_idx);
  *first_pt = points()[*first_idx];
  *last_pt = points()[*last_idx];
}

void Polygon2d::ExtremePoints(Vec2d direction_vec, int *first,
                              int *last) const {
  QCHECK_GE(points_.size(), 3);
  QCHECK(first != nullptr);
  QCHECK(last != nullptr);

  double min_proj = std::numeric_limits<double>::infinity();
  double max_proj = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < num_points_; ++i) {
    const double proj = points_[i].dot(direction_vec);
    if (proj < min_proj) {
      min_proj = proj;
      *first = i;
    }
    if (proj > max_proj) {
      max_proj = proj;
      *last = i;
    }
  }
}

Box2d Polygon2d::BoundingBoxWithHeading(const double heading) const {
  QCHECK_GE(points_.size(), 3);
  const Vec2d direction_vec = Vec2d::UnitFromAngle(heading);
  Vec2d px1;
  Vec2d px2;
  Vec2d py1;
  Vec2d py2;
  ExtremePoints(heading, &px1, &px2);
  ExtremePoints(heading - M_PI_2, &py1, &py2);
  const double x1 = px1.dot(direction_vec);
  const double x2 = px2.dot(direction_vec);
  const double y1 = py1.CrossProd(direction_vec);
  const double y2 = py2.CrossProd(direction_vec);
  return Box2d(
      (x1 + x2) / 2.0 * direction_vec +
          (y1 + y2) / 2.0 * Vec2d(direction_vec.y(), -direction_vec.x()),
      heading, x2 - x1, y2 - y1);
}

Box2d Polygon2d::MinAreaBoundingBox() const {
  QCHECK_GE(points_.size(), 3);
  if (!is_convex_) {
    Polygon2d convex_polygon;
    ComputeConvexHull(points_, &convex_polygon);
    QCHECK(convex_polygon.is_convex());
    return convex_polygon.MinAreaBoundingBox();
  }
  double min_area = std::numeric_limits<double>::infinity();
  double min_area_at_heading = 0.0;
  int left_most = 0;
  int right_most = 0;
  int top_most = 0;
  for (int i = 0; i < num_points_; ++i) {
    const auto &line_segment = line_segments_[i];
    double proj = 0.0;
    double min_proj = line_segment.ProjectOntoUnit(points_[left_most]);
    while ((proj = line_segment.ProjectOntoUnit(points_[Prev(left_most)])) <
           min_proj) {
      min_proj = proj;
      left_most = Prev(left_most);
    }
    while ((proj = line_segment.ProjectOntoUnit(points_[Next(left_most)])) <
           min_proj) {
      min_proj = proj;
      left_most = Next(left_most);
    }
    double max_proj = line_segment.ProjectOntoUnit(points_[right_most]);
    while ((proj = line_segment.ProjectOntoUnit(points_[Prev(right_most)])) >
           max_proj) {
      max_proj = proj;
      right_most = Prev(right_most);
    }
    while ((proj = line_segment.ProjectOntoUnit(points_[Next(right_most)])) >
           max_proj) {
      max_proj = proj;
      right_most = Next(right_most);
    }
    double prod = 0.0;
    double max_prod = line_segment.ProductOntoUnit(points_[top_most]);
    while ((prod = line_segment.ProductOntoUnit(points_[Prev(top_most)])) >
           max_prod) {
      max_prod = prod;
      top_most = Prev(top_most);
    }
    while ((prod = line_segment.ProductOntoUnit(points_[Next(top_most)])) >
           max_prod) {
      max_prod = prod;
      top_most = Next(top_most);
    }
    const double area = max_prod * (max_proj - min_proj);
    if (area < min_area) {
      min_area = area;
      min_area_at_heading = line_segment.heading();
    }
  }
  return BoundingBoxWithHeading(min_area_at_heading);
}

Polygon2d Polygon2d::ExpandByDistance(const double distance) const {
  if (!is_convex_) {
    Polygon2d convex_polygon;
    ComputeConvexHull(points_, &convex_polygon);
    QCHECK(convex_polygon.is_convex());
    return convex_polygon.ExpandByDistance(distance);
  }
  const double kMinAngle = 0.1;
  std::vector<Vec2d> points;
  for (int i = 0; i < num_points_; ++i) {
    const double start_angle = line_segments_[Prev(i)].heading() - M_PI_2;
    const double end_angle = line_segments_[i].heading() - M_PI_2;
    const double diff = NormalizeAngle(end_angle - start_angle);
    if (diff <= kEpsilon) {
      points.push_back(points_[i] +
                       Vec2d::UnitFromAngle(start_angle) * distance);
    } else {
      const int count = static_cast<int>(diff / kMinAngle) + 1;
      for (int k = 0; k <= count; ++k) {
        const double angle = start_angle + diff * static_cast<double>(k) /
                                               static_cast<double>(count);
        points.push_back(points_[i] + Vec2d::UnitFromAngle(angle) * distance);
      }
    }
  }
  Polygon2d new_polygon;
  QCHECK(ComputeConvexHull(points, &new_polygon));
  return new_polygon;
}

Polygon2d Polygon2d::ExtrudeAlongVector(const Vec2d vec) const {
  std::vector<Vec2d> points;
  points.reserve(num_points_ + 2);
  const double vec_angle = vec.Angle();
  for (int i = 0; i < num_points_; ++i) {
    const double start_angle =
        NormalizeAngle(line_segments_[Prev(i)].heading() - vec_angle);
    const double end_angle =
        NormalizeAngle(line_segments_[i].heading() - vec_angle);
    if (start_angle < 0.0 && end_angle >= 0.0) {
      points.push_back(points_[i]);
    } else if (start_angle >= 0.0 && end_angle < 0.0) {
      points.push_back(points_[i] + vec);
    }
    points.push_back(points_[i] + (end_angle > 0.0 ? vec : Vec2d::Zero()));
  }
  return Polygon2d(std::move(points));
}

Polygon2d Polygon2d::Transform(Vec2d center, double cos_angle, double sin_angle,
                               Vec2d translation) const {
  // Use large error tolerance to accept approximately computed cos/sin values.
  DCHECK_GE(Sqr(cos_angle) + Sqr(sin_angle), 1.0 - 1e-4);
  DCHECK_LE(Sqr(cos_angle) + Sqr(sin_angle), 1.0 + 1e-4);
  std::vector<Vec2d> points;
  points.reserve(num_points());
  const Vec2d new_center = center + translation;
  for (Vec2d pt : points_) {
    points.emplace_back(Vec2d(pt - center).Rotate(cos_angle, sin_angle) +
                        new_center);
  }
  return Polygon2d(std::move(points), is_convex());
}

std::string Polygon2d::DebugString() const {
  std::string points_str;
  for (const auto &point : points_) {
    points_str += point.DebugString();
  }
  return absl::StrCat("polygon2d (  num_points = ", num_points_, "  points = (",
                      points_str, " )  ", is_convex_ ? "convex" : "non-convex",
                      "  area = ", area_, " )");
}

Segment2d Polygon2d::GetPrincipalAxis() const {
  const auto box = MinAreaBoundingBox();
  double half_len = box.half_length();
  Vec2d heading(box.cos_heading(), box.sin_heading());
  if (box.length() < box.width()) {
    heading = heading.Perp();
    half_len = box.half_width();
  }
  return Segment2d(box.center() + heading * half_len,
                   box.center() - heading * half_len);
}

}  // namespace qcraft
