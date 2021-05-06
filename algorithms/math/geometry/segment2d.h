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

#ifndef ONBOARD_MATH_GEOMETRY_SEGMENT2D_H_
#define ONBOARD_MATH_GEOMETRY_SEGMENT2D_H_

#include <algorithm>
#include <optional>
#include <string>
#include <utility>

#include "onboard/math/vec.h"

namespace qcraft {

/**
 * @class Segment2d
 * @brief Line segment in 2-D.
 */
class Segment2d {
 public:
  /**
   * @brief Empty constructor.
   */
  Segment2d() { unit_direction_ = Vec2d(1, 0); }

  /**
   * @brief Constructor with start point and end point.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  Segment2d(const Vec2d &start, const Vec2d &end);

  /**
   * @brief Get the start point.
   * @return The start point of the line segment.
   */
  const Vec2d &start() const { return start_; }

  /**
   * @brief Get the end point.
   * @return The end point of the line segment.
   */
  const Vec2d &end() const { return end_; }

  /**
   * @brief Get the unit direction from the start point to the end point.
   * @return The start point of the line segment.
   */
  const Vec2d &unit_direction() const { return unit_direction_; }

  /**
   * @brief Get the center of the line segment.
   * @return The center of the line segment.
   */
  Vec2d center() const { return (start_ + end_) / 2.0; }

  /**
   * @brief Get the heading of the line segment.
   * @return The heading, which is the angle between unit direction and x-axis.
   */
  // NOTE: not thread safe!
  double heading() const {
    // Lazy computation.
    if (!heading_) {
      heading_ = unit_direction_.Angle();
    }
    return *heading_;
  }

  /**
   * @brief Get the cosine of the heading.
   * @return The cosine of the heading.
   */
  double cos_heading() const { return unit_direction_.x(); }

  /**
   * @brief Get the sine of the heading.
   * @return The sine of the heading.
   */
  double sin_heading() const { return unit_direction_.y(); }

  /**
   * @brief Get the length of the line segment.
   * @return The length of the line segment.
   */
  double length() const { return length_; }

  /**
   * @brief Get the square of length of the line segment.
   * @return The square of length of the line segment.
   */
  double length_sqr() const { return length_ * length_; }

  // Returns the min x.
  double min_x() const { return std::min(start_.x(), end_.x()); }
  // Returns the max x.
  double max_x() const { return std::max(start_.x(), end_.x()); }
  // Returns the min y.
  double min_y() const { return std::min(start_.y(), end_.y()); }
  // Returns the max y.
  double max_y() const { return std::max(start_.y(), end_.y()); }

  /**
   * @brief Compute the signed distance from a point in 2-D to the line of the
   * segment with a 2-D point on the right of the segment being positive and
   * on the left being negative.
   * @param point The point to compute the distance to.
   * @return The signed distance from a 2-D point to the segment line.
   */
  double SignedDistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D.
   * @param point The point to compute the distance to.
   * @return The shortest distance from points on the line segment to point.
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D, and get the nearest point on the line segment.
   * @param point The point to compute the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  double DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Computes the shortest distance between two segments.
   * @param segment The segment to compute the distance to.
   * @return The shortest distance.
   */
  double DistanceTo(const Segment2d &segment) const;

  /**
   * @brief Computes the shortest distance between two segments.
   * @param segment The segment to compute the distance to.
   * @param nearest_pt The nearest point on this segment.
   * @param other_nearest_pt The nearest point on the other segment.
   * @return The shortest distance.
   */
  double DistanceTo(const Segment2d &segment, Vec2d *nearest_pt,
                    Vec2d *other_nearest_pt) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D.
   * @param point The point to compute the squared of the distance to.
   * @return The square of the shortest distance from points
   *         on the line segment to the input point.
   */
  double DistanceSquareTo(const Vec2d &point) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D,
   *        and get the nearest point on the line segment.
   * @param point The point to compute the squared of the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  double DistanceSquareTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  /**
   * @brief Check if a point is within the line segment.
   * @param point The point to check if it is within the line segment.
   * @return Whether the input point is within the line segment or not.
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Check if the line segment has an intersect
   *        with another line segment in 2-D.
   * @param other_segment The line segment to check if it has an intersect.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool HasIntersect(const Segment2d &other_segment) const;

  /**
   * @brief Compute the intersect with another line segment in 2-D if any.
   * @param other_segment The line segment to compute the intersect.
   * @param point the computed intersect between the line segment and
   *        the input other_segment.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool GetIntersect(const Segment2d &other_segment, Vec2d *const point) const;

  /**
   * @brief Compute the projection of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the projection onto the line segment.
   * @return The projection of the vector, which is from the start point of
   *         the line segment to the input point, onto the unit direction.
   */
  double ProjectOntoUnit(const Vec2d &point) const;

  /**
   * @brief Compute the cross product of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the cross product onto the line segment.
   * @return The cross product of the unit direction and
   *         the vector, which is from the start point of
   *         the line segment to the input point.
   */
  double ProductOntoUnit(const Vec2d &point) const;

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the straight line
   *        expanded from the line segment.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the straight line expanded from the line segment.
   * @return The distance from the input point to the perpendicular foot.
   */
  double GetPerpendicularFoot(const Vec2d &point,
                              Vec2d *const foot_point) const;

  /**
   * @brief Get the debug string including the essential information.
   * @return Information of the line segment for debugging.
   */
  std::string DebugString() const;

 private:
  static constexpr double kEpsilon = 1e-10;

  static double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                          const Vec2d &end_point_2) {
    return Vec2d(end_point_1 - start_point)
        .CrossProd(end_point_2 - start_point);
  }

  static bool IsWithin(double val, double bound1, double bound2) {
    if (bound1 > bound2) {
      std::swap(bound1, bound2);
    }
    return val >= bound1 - kEpsilon && val <= bound2 + kEpsilon;
  }

  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  mutable std::optional<double> heading_;
  double length_ = 0.0;
};

inline Segment2d::Segment2d(const Vec2d &start, const Vec2d &end)
    : start_(start), end_(end) {
  const double dx = end_.x() - start_.x();
  const double dy = end_.y() - start_.y();
  length_ = Hypot(dx, dy);
  unit_direction_ =
      (length_ <= kEpsilon ? Vec2d(0, 0) : Vec2d(dx / length_, dy / length_));
}

inline double Segment2d::SignedDistanceTo(const Vec2d &point) const {
  if (length_ <= kEpsilon) return point.DistanceTo(start_);
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  return x0 * unit_direction_.y() - y0 * unit_direction_.x();
}

inline double Segment2d::DistanceTo(const Vec2d &point) const {
  if (length_ <= kEpsilon) {
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

inline bool Segment2d::IsPointIn(const Vec2d &point) const {
  if (length_ <= kEpsilon) {
    return std::abs(point.x() - start_.x()) <= kEpsilon &&
           std::abs(point.y() - start_.y()) <= kEpsilon;
  }
  const double prod = CrossProd(point, start_, end_);
  if (std::abs(prod) > kEpsilon) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_SEGMENT2D_H_
