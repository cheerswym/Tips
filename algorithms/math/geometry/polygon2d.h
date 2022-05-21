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

#ifndef ONBOARD_MATH_GEOMETRY_POLYGON2D_H_
#define ONBOARD_MATH_GEOMETRY_POLYGON2D_H_

#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/box2d.h"

namespace qcraft {

/**
 * @class Polygon2d
 * @brief The class of polygon in 2-D.
 */
class Polygon2d {
 public:
  /**
   * @brief Empty constructor.
   */
  Polygon2d() = default;

  /**
   * @brief Constructor which takes a box.
   * @param box The box to construct the polygon.
   */
  explicit Polygon2d(const Box2d &box);

  /**
   * @brief Constructor which takes a vector of points as its vertices.
   * @param points The points to construct the polygon.
   * @param `is_convex` indicates if the polygon is a convex polygon.
   */
  explicit Polygon2d(std::vector<Vec2d> points);

  // Similar to the above constructor, but pre-specifies whether the polygon is
  // a convex polygon. It is faster than the above function.
  Polygon2d(std::vector<Vec2d> points, bool is_convex);

  /**
   * @brief Get the vertices of the polygon.
   * @return The vertices of the polygon.
   */
  const std::vector<Vec2d> &points() const { return points_; }

  /**
   * @brief Get the edges of the polygon.
   * @return The edges of the polygon.
   */
  const std::vector<Segment2d> &line_segments() const { return line_segments_; }

  /**
   * @brief Get the number of vertices of the polygon.
   * @return The number of vertices of the polygon.
   */
  int num_points() const { return num_points_; }

  /**
   * @brief Check if the polygon is convex.
   * @return Whether the polygon is convex or not.
   */
  bool is_convex() const { return is_convex_; }

  // Test if the polygon is self intersecting, i.e., has non-neighboring edges
  // that are overlapping.
  bool IsSelfIntersecting() const;

  /**
   * @brief Get the area of the polygon.
   * @return The area of the polygon.
   */
  double area() const { return area_; }

  Vec2d centroid() const {
    return std::accumulate(points_.begin(), points_.end(), Vec2d(0.0, 0.0)) /
           points_.size();
  }

  /**
   * @brief Compute the distance from a point to the boundary of the polygon.
   *        This distance is equal to the minimal distance from the point
   *        to the edges of the polygon.
   * @param point The point to compute whose distance to the polygon.
   * @return The distance from the point to the polygon's boundary.
   */
  double DistanceToBoundary(const Vec2d &point) const;

  /**
   * @brief Compute the distance from a point to the polygon. If the point is
   *        within the polygon, return 0. Otherwise, this distance is
   *        the minimal distance from the point to the edges of the polygon.
   * @param point The point to compute whose distance to the polygon.
   * @return The distance from the point to the polygon.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the distance from a line segment to the polygon.
   *        If the line segment is within the polygon, or it has intersect with
   *        the polygon, return 0. Otherwise, this distance is
   *        the minimal distance between the distances from the two ends
   *        of the line segment to the polygon.
   * @param line_segment The line segment to compute whose distance to
   *        the polygon.
   * @return The distance from the line segment to the polygon.
   */
  double DistanceTo(const Segment2d &line_segment) const;

  /**
   * @brief Compute the distance from a box to the polygon.
   *        If the box is within the polygon, or it has overlap with
   *        the polygon, return 0. Otherwise, this distance is
   *        the minimal distance among the distances from the edges
   *        of the box to the polygon.
   * @param box The box to compute whose distance to the polygon.
   * @return The distance from the box to the polygon.
   */
  double DistanceTo(const Box2d &box) const;

  /**
   * @brief Compute the distance from another polygon to the polygon.
   *        If the other polygon is within this polygon, or it has overlap with
   *        this polygon, return 0. Otherwise, this distance is
   *        the minimal distance among the distances from the edges
   *        of the other polygon to this polygon.
   * @param polygon The polygon to compute whose distance to this polygon.
   * @return The distance from the other polygon to this polygon.
   */
  double DistanceTo(const Polygon2d &polygon) const;

  /**
   * @brief Compute the square of distance from a point to the polygon.
   *        If the point is within the polygon, return 0. Otherwise,
   *        this square of distance is the minimal square of distance from
   *        the point to the edges of the polygon.
   * @param point The point to compute whose square of distance to the polygon.
   * @return The square of distance from the point to the polygon.
   */
  double DistanceSquareTo(const Vec2d &point) const;

  /**
   * @brief Check if a point is within the polygon.
   * @param point The target point. To check if it is within the polygon.
   * @return Whether a point is within the polygon or not.
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Check if a point is on the boundary of the polygon.
   * @param point The target point. To check if it is on the boundary
   *        of the polygon.
   * @return Whether a point is on the boundary of the polygon or not.
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Check if the polygon contains a line segment.
   * @param line_segment The target line segment. To check if the polygon
   *        contains it.
   * @return Whether the polygon contains the line segment or not.
   */
  bool Contains(const Segment2d &line_segment) const;

  /**
   * @brief Check if the polygon contains another polygon.
   * @param polygon The target polygon. To check if this polygon contains it.
   * @return Whether this polygon contains another polygon or not.
   */
  bool Contains(const Polygon2d &polygon) const;

  /**
   * @brief Compute the convex hull of a group of points.
   * @param points The target points. To compute the convex hull of them.
   * @param polygon The convex hull of the points.
   * @return If successfully compute the convex hull.
   */
  static bool ComputeConvexHull(absl::Span<const Vec2d> points,
                                Polygon2d *const polygon);

  // Similiar to last function, but only returns a vector of points, without
  // constructing a Polygon2d.
  static bool ComputeConvexHullPoints(absl::Span<const Vec2d> points,
                                      std::vector<Vec2d> *convex_hull_points);

  static Polygon2d MergeTwoBoxes(const Box2d &first_box,
                                 const Box2d &second_box);

  static Polygon2d MergeBoxes(const std::vector<Box2d> &boxes);

  static Polygon2d MergeTwoPolygons(const Polygon2d &first_polygon,
                                    const Polygon2d &second_polygon);

  // Test if a list of provided points are convex points.
  static bool AreConvexHullPoints(absl::Span<const Vec2d> points);

  /**
   * @brief Check if a line segment has overlap with this polygon.
   * @param line_segment The target line segment. To check if it has
   *        overlap with this polygon.
   * @return Whether the target line segment has overlap with this
   *         polygon or not.
   */
  bool HasOverlap(const Segment2d &line_segment) const;

  /**
   * @brief Check if a box has overlap with this polygon.
   * @param box2d The target box. To check if it has overlap with this polygon.
   * @return Whether the target box has overlap with this polygon or not.
   */
  bool HasOverlap(const Box2d &box) const;

  /**
   * @brief Get the overlap of a line segment and this polygon. If they have
   *        overlap, output the two ends of the overlapped line segment.
   * @param line_segment The target line segment. To get its overlap with
   *         this polygon.
   * @param first First end of the overlapped line segment.
   * @param second Second end of the overlapped line segment.
   * @return If the target line segment has overlap with this polygon.
   */
  bool GetOverlap(const Segment2d &line_segment, Vec2d *const first,
                  Vec2d *const last) const;

  /**
   * @brief Get all vertices of the polygon
   * @param All vertices of the polygon
   */
  void GetAllVertices(std::vector<Vec2d> *const vertices) const {
    *vertices = points_;
  }

  /**
   * @brief Get all vertices of the polygon
   */
  const std::vector<Vec2d> &GetAllVertices() const { return points_; }

  /**
   * @brief Get all overlapped line segments of a line segment and this polygon.
   *        There are possibly multiple overlapped line segments if this
   *        polygon is not convex.
   * @param line_segment The target line segment. To get its all overlapped
   *        line segments with this polygon.
   * @return A group of overlapped line segments.
   */
  std::vector<Segment2d> GetAllOverlaps(const Segment2d &line_segment) const;

  /**
   * @brief Check if this polygon has overlap with another polygon.
   * @param polygon The target polygon. To check if it has overlap
   *        with this polygon.
   * @return If this polygon has overlap with another polygon.
   */
  bool HasOverlap(const Polygon2d &polygon) const;

  // Only compute overlaps between two convex polygons.
  /**
   * @brief Compute the overlap of this polygon and the other polygon if any.
   *        Note: this function only works for computing overlap between
   *        two convex polygons.
   * @param other_polygon The target polygon. To compute its overlap with
   *        this polygon.
   * @param overlap_polygon The overlapped polygon.
   * @param If there is a overlapped polygon.
   */
  bool ComputeOverlap(const Polygon2d &other_polygon,
                      Polygon2d *const overlap_polygon) const;

  // Returns the circle radius of the polygon.
  double CircleRadius() const { return aabox_.half_diagonal(); }

  // Returns the circle center of the polygon.
  Vec2d CircleCenter() const { return aabox_.center(); }

  /**
   * @brief Get the axis-aligned bound box of the polygon.
   * @return The axis-aligned bound box of the polygon.
   */
  const AABox2d &AABoundingBox() const { return aabox_; }

  /**
   * @brief Get the bound box according to a heading.
   * @param heading The specified heading of the bounding box.
   * @return The bound box according to the specified heading.
   */
  [[nodiscard]] Box2d BoundingBoxWithHeading(const double heading) const;

  /**
   * @brief Get the bounding box with the minimal area.
   * @return The bounding box with the minimal area.
   */
  [[nodiscard]] Box2d MinAreaBoundingBox() const;

  /**
   * @brief Get the extreme points along a heading direction.
   * @param heading The specified heading.
   * @param first The point on the boundary of this polygon with the minimal
   *        projection onto the heading direction.
   * @param last The point on the boundary of this polygon with the maximal
   *        projection onto the heading direction.
   */
  void ExtremePoints(const double heading, Vec2d *const first,
                     Vec2d *const last) const;

  // Same as above but saves a sin/cos call if the input is already a direction
  // vector.
  void ExtremePoints(Vec2d direction_vec, Vec2d *first, Vec2d *last) const;

  // Same as above but returns the vertex indices found.
  void ExtremePoints(Vec2d direction_vec, int *first, int *last) const;

  // Same as above but returns the vertex indices and pts found.
  void ExtremePoints(Vec2d direction_vec, int *first_idx, int *last_idx,
                     Vec2d *first_pt, Vec2d *last_pt) const;

  /**
   * @brief Expand this polygon by a distance.
   * @param distance The specified distance. To expand this polygon by it.
   * @return The polygon after expansion.
   */
  [[nodiscard]] Polygon2d ExpandByDistance(const double distance) const;

  [[nodiscard]] Polygon2d ExtrudeAlongVector(const Vec2d vec) const;

  /**
   * @brief Transform the polygon. It first rotate all the points by a rotation
   * angle around center point, then translate all the points by vector
   * `translation`.
   * @param center The center of the polygon point.
   * @param cos_angle The cosine value of rotation angle.
   * @param sin_angle The sine value of rotation angle.
   * @param translation The translation vector.
   */
  [[nodiscard]] Polygon2d Transform(Vec2d center, double cos_angle,
                                    double sin_angle, Vec2d translation) const;

  /**
   * @brief Get a string containing essential information about the polygon
   *        for debugging purpose.
   * @return Essential information about the polygon for debugging purpose.
   */
  std::string DebugString() const;

  // Returns a debug string that does not have data presentation loss.
  std::string DebugStringFullPrecision() const {
    return absl::StrCat(qcraft::DebugStringFullPrecision(points_),
                        ", /*is_convex=*/", is_convex());
  }

  // Supports to construct a `Polygon2d` from a container that has a list of
  // points. Each point should have `x()` and `y()` accessors.
  template <typename Container>
  static std::optional<Polygon2d> FromPoints(const Container &points,
                                             bool is_convex) {
    if (points.size() < 3) return std::nullopt;
    std::vector<Vec2d> pts;
    pts.reserve(points.size());
    for (const auto &pt : points) {
      pts.emplace_back(pt.x(), pt.y());
    }
    return Polygon2d(std::move(pts), is_convex);
  }

  // Fit a polygon's principal axis
  Segment2d GetPrincipalAxis() const;

  double min_x() const { return aabox_.min_x(); }
  double max_x() const { return aabox_.max_x(); }
  double min_y() const { return aabox_.min_y(); }
  double max_y() const { return aabox_.max_y(); }

 protected:
  void BuildFromPoints();
  int Next(int at) const;
  int Prev(int at) const;

  static bool ClipConvexHull(const Segment2d &line_segment,
                             std::vector<Vec2d> *const points);

  std::vector<Vec2d> points_;
  int num_points_ = 0;
  std::vector<Segment2d> line_segments_;
  bool is_convex_ = false;
  double area_ = 0.0;
  AABox2d aabox_;
};

inline bool Polygon2d::IsPointOnBoundary(const Vec2d &point) const {
  QCHECK_GE(points_.size(), 3);
  return std::any_of(
      line_segments_.begin(), line_segments_.end(),
      [&](const Segment2d &poly_seg) { return poly_seg.IsPointIn(point); });
}

inline bool Polygon2d::IsPointIn(const Vec2d &point) const {
  if (!aabox_.IsPointIn(point)) return false;

  QCHECK_GE(points_.size(), 3);
  if (IsPointOnBoundary(point)) {
    return true;
  }
  int j = num_points_ - 1;
  int c = 0;
  for (int i = 0; i < num_points_; ++i) {
    if ((points_[i].y() > point.y()) != (points_[j].y() > point.y())) {
      const double side =
          Vec2d(points_[i] - point).CrossProd(points_[j] - point);
      if (points_[i].y() < points_[j].y() ? side > 0.0 : side < 0.0) {
        ++c;
      }
    }
    j = i;
  }
  return c & 1;
}

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_POLYGON2D_H_
