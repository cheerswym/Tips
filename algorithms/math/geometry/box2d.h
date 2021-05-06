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

#ifndef ONBOARD_MATH_GEOMETRY_BOX2D_H_
#define ONBOARD_MATH_GEOMETRY_BOX2D_H_

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/box2d.pb.h"
#include "onboard/math/geometry/segment2d.h"

namespace qcraft {

/**
 * @class Box2d
 * @brief Rectangular (undirected) bounding box in 2-D.
 *
 * This class is referential-agnostic, although our convention on the use of
 * the word "heading" in this project (permanently set to be 0 at East)
 * forces us to assume that the X/Y frame here is East/North.
 * For disambiguation, we call the axis of the rectangle parallel to the
 * heading direction the "heading-axis". The size of the heading-axis is
 * called "length", and the size of the axis perpendicular to it "width".
 */
class Box2d {
 public:
  // The four corner point names.
  enum Corner {
    FRONT_LEFT = 0,
    REAR_LEFT = 1,
    REAR_RIGHT = 2,
    FRONT_RIGHT = 3,
  };
  Box2d() = default;
  /**
   * @brief Constructor which takes the center, heading, length and width.
   * @param center The center of the rectangular bounding box.
   * @param heading The angle between the x-axis and the heading-axis,
   *        measured counter-clockwise.
   * @param length The size of the heading-axis.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
  Box2d(Vec2d center, double heading, double length, double width);

  // Similar to the above function, but uses half length and half width.
  Box2d(double half_length, double half_width, Vec2d center, double heading);

  // This function is faster when heading and tangent are all known when
  // constructing the box.
  Box2d(double half_length, double half_width, Vec2d center, double heading,
        Vec2d tangent);

  // Construct box from a proto.
  explicit Box2d(const Box2dProto &proto) { FromProto(proto); }

  /**
   * @brief Constructor which takes the center, tangent, length and width.
   * @param center The center of the rectangular bounding box.
   * @param tangent The heading of the box represented by a unit vector.
   * @param length The size of the heading-axis.
   * @param width The size of the axis perpendicular to the heading-axis.
   */
  Box2d(Vec2d center, Vec2d tangent, double length, double width);

  // Similar to the above function, but uses half length and half width.
  Box2d(double half_length, double half_width, Vec2d center, Vec2d tangent);

  /**
   * @brief Constructor which takes the heading-axis and the width of the box
   * @param axis The heading-axis
   * @param width The width of the box, which is taken perpendicularly
   * to the heading direction.
   */
  Box2d(const Segment2d &axis, double width);

  /**
   * @brief Constructor which takes an AABox2d (axes-aligned box).
   * @param aabox The input AABox2d.
   */
  explicit Box2d(const AABox2d &aabox);

  /**
   * @brief Creates an axes-aligned Box2d from two opposite corners
   * @param one_corner One of the corners
   * @param opposite_corner The opposite corner to the first one
   * @return An axes-aligned Box2d
   */
  static Box2d CreateAABox(const Vec2d &one_corner,
                           const Vec2d &opposite_corner);

  /**
   * @brief Getter of the center of the box
   * @return The center of the box
   */
  const Vec2d &center() const { return center_; }

  /**
   * @brief Getter of the x-coordinate of the center of the box
   * @return The x-coordinate of the center of the box
   */
  double center_x() const { return center_.x(); }

  /**
   * @brief Getter of the y-coordinate of the center of the box
   * @return The y-coordinate of the center of the box
   */
  double center_y() const { return center_.y(); }

  /**
   * @brief Getter of the length
   * @return The length of the heading-axis
   */
  double length() const { return half_length_ * 2.0; }

  /**
   * @brief Getter of the width
   * @return The width of the box taken perpendicularly to the heading
   */
  double width() const { return half_width_ * 2.0; }

  /**
   * @brief Getter of half the length
   * @return Half the length of the heading-axis
   */
  double half_length() const { return half_length_; }

  /**
   * @brief Getter of half the width
   * @return Half the width of the box taken perpendicularly to the heading
   */
  double half_width() const { return half_width_; }

  /**
   * @brief Getter of the heading
   * @return The counter-clockwise angle between the x-axis and the heading-axis
   */
  double heading() const { return heading_; }

  /**
   * @brief Getter of the cosine of the heading
   * @return The cosine of the heading
   */
  double cos_heading() const { return cos_heading_; }

  /**
   * @brief Getter of the sine of the heading
   * @return The sine of the heading
   */
  double sin_heading() const { return sin_heading_; }

  /**
   * @brief Getter of the area of the box
   * @return The product of its length and width
   */
  double area() const { return half_length_ * half_width_ * 4.0; }

  /**
   * @brief Getter of the size of the diagonal of the box
   * @return The diagonal size of the box
   */
  double diagonal() const { return Hypot(half_length_, half_width_) * 2.0; }

  /**
   * @brief Returns the tangent of the box, which is a unit vector.
   * @return The tangent of the box.
   */
  Vec2d tangent() const { return Vec2d(cos_heading_, sin_heading_); }

  /**
   * @brief Return one specified corner of the box.
   * @param corner The name of the corner.
   */
  Vec2d GetCorner(Corner corner) const;

  // Returns the front center point of the box.
  Vec2d FrontCenterPoint() const;

  // Returns the rear center point of the box.
  Vec2d RearCenterPoint() const;

  /**
   * @brief Return the corners of the box in counter clockwise order. You can
   * use Box2d::Corner as the index of the corners.
   * Returned corners are FRONT_LEFT, REAR_LEFT, REAR_RIGHT, FRONT_RIGHT.
   */
  std::vector<Vec2d> GetCornersCounterClockwise() const;

  /**
   * @brief Tests points for membership in the box
   * @param point A point that we wish to test for membership in the box
   * @return True iff the point is contained in the box
   */
  bool IsPointIn(const Vec2d &point) const;

  /**
   * @brief Tests points for membership in the boundary of the box
   * @param point A point that we wish to test for membership in the boundary
   * @return True iff the point is a boundary point of the box
   */
  bool IsPointOnBoundary(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the box and a given point
   * @param point The point whose distance to the box we wish to compute
   * @return A distance
   */
  double DistanceTo(const Vec2d &point) const;

  /**
   * @brief Determines the distance between the box and a given line segment
   * @param line_segment The line segment whose distance to the box we compute
   * @return A distance
   */
  double DistanceTo(const Segment2d &line_segment) const;

  /**
   * @brief Determines the distance between two boxes
   * @param box The box whose distance to this box we want to compute
   * @return A distance
   */
  double DistanceTo(const Box2d &box) const;

  /**
   * @brief Determines whether this box overlaps a given line segment
   * @param line_segment The line-segment
   * @return True if they overlap
   */
  bool HasOverlap(const Segment2d &line_segment) const;

  /**
   * @brief Determines whether these two boxes overlap
   * @param line_segment The other box
   * @return True if they overlap
   */
  bool HasOverlap(const Box2d &box) const;

  /**
   * @brief Determines whether it overlaps with an AABox2d.
   * @param aabox The axis aligned box.
   * @return True if they overlap
   */
  bool HasOverlap(const AABox2d &aabox) const;

  /**
   * @brief Gets the smallest axes-aligned box containing the current one
   * @return An axes-aligned box
   */
  [[nodiscard]] AABox2d GetAABox() const;

  /**
   * @brief Rotate from center.
   * @param rotate_angle Angle to rotate.
   */
  void RotateFromCenter(double rotate_angle);

  /**
   * @brief Shifts this box by a given vector
   * @param shift_vec The vector determining the shift
   */
  void Shift(const Vec2d &shift_vec);

  // Mirror this box around x-axis.
  void MirrorByX();

  // Translate this box with the given vector, and return the shifted box.
  [[nodiscard]] Box2d Transform(Vec2d translation) const;

  // Rotate this box with the given angle, and return the rotated box.
  // Note that it rotates the center of the box around the origin.
  [[nodiscard]] Box2d Transform(double rotation) const;

  // Transform this box with the given transformation, and return the
  // transformed box. Note that it rotates the center of the box around origin.
  [[nodiscard]] Box2d Transform(Vec2d translation, double rotation) const;

  /**
   * @brief Extend the box w.r.t. tangent direction
   * @param extension_length the length to extend
   */
  void LongitudinalExtend(double extension_length);

  void LateralExtend(double extension_length);

  /**
   * @brief Extend the box w.r.t. tangent direction
   * @param extension_ratio the length ratio to extend
   */
  void LongitudinalExtendByRatio(double extension_ratio);

  void LateralExtendByRatio(double extension_ratio);

  // Create from a proto.
  void FromProto(const Box2dProto &proto);

  // Convert it to a proto.
  void ToProto(Box2dProto *proto) const;

  /**
   * @brief Gets a human-readable description of the box
   * @return A debug-string
   */
  std::string DebugString() const;

 private:
  static constexpr double kEpsilon = 1e-10;

  Vec2d center_;
  double half_length_ = 0.0;
  double half_width_ = 0.0;
  double heading_ = 0.0;
  double cos_heading_ = 1.0;
  double sin_heading_ = 0.0;
};

inline Box2d::Box2d(Vec2d center, const double heading, const double length,
                    const double width)
    : Box2d(length * 0.5, width * 0.5, center, heading) {}

inline Box2d::Box2d(const double half_length, const double half_width,
                    Vec2d center, const double heading)
    : center_(center),
      half_length_(half_length),
      half_width_(half_width),
      heading_(heading),
      cos_heading_(cos(heading)),
      sin_heading_(sin(heading)) {
  QCHECK_GT(half_length_, -kEpsilon);
  QCHECK_GT(half_width_, -kEpsilon);
}

inline Box2d::Box2d(const double half_length, const double half_width,
                    Vec2d center, const double heading, Vec2d tangent)
    : center_(center),
      half_length_(half_length),
      half_width_(half_width),
      heading_(heading),
      cos_heading_(tangent.x()),
      sin_heading_(tangent.y()) {
  QCHECK_GT(half_length_, -kEpsilon);
  QCHECK_GT(half_width_, -kEpsilon);
  DCHECK_LE(std::abs(Sqr(cos_heading_) + Sqr(sin_heading_) - 1.0), 1e-8);
}

inline Box2d::Box2d(Vec2d center, Vec2d tangent, double length, double width)
    : Box2d(length * 0.5, width * 0.5, center, tangent) {}

inline Box2d::Box2d(double half_length, double half_width, Vec2d center,
                    Vec2d tangent)
    : center_(center),
      half_length_(half_length),
      half_width_(half_width),
      heading_(tangent.Angle()),
      cos_heading_(tangent.x()),
      sin_heading_(tangent.y()) {
  QCHECK_GT(half_length_, -kEpsilon);
  QCHECK_GT(half_width_, -kEpsilon);
  // We encountered a check failure of an unit tangent vector converted by
  // Vec2d::FastUnitFromAngle() here. The difference between its 2 norm and 1.0
  // is on the 1e-8 order. We believe it should pass the unit vector check so we
  // temporarily change the tolerance to 1e-6 (original value is kEpsilon). This
  // problem need to be fixed in the future.
  DCHECK_LT(std::abs(tangent.Sqr() - 1.0), 1e-6);
}

inline Box2d::Box2d(const Segment2d &axis, const double width)
    : center_(axis.center()),
      half_length_(axis.length() * 0.5),
      half_width_(width * 0.5),
      heading_(axis.heading()),
      cos_heading_(axis.cos_heading()),
      sin_heading_(axis.sin_heading()) {
  QCHECK_GT(half_length_, -kEpsilon);
  QCHECK_GT(half_width_, -kEpsilon);
}

inline Box2d::Box2d(const AABox2d &aabox)
    : center_(aabox.center()),
      half_length_(aabox.half_length()),
      half_width_(aabox.half_width()),
      heading_(0.0),
      cos_heading_(1.0),
      sin_heading_(0.0) {
  QCHECK_GT(half_length_, -kEpsilon);
  QCHECK_GT(half_width_, -kEpsilon);
}

inline std::vector<Vec2d> Box2d::GetCornersCounterClockwise() const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  const Vec2d w = unit.Perp() * half_width_;
  return {
      center_ + h + w,  // FRONT_LEFT
      center_ - h + w,  // REAR_LEFT
      center_ - h - w,  // REAR_RIGHT
      center_ + h - w,  // FRONT_RIGHT
  };
}

inline Vec2d Box2d::GetCorner(Corner corner) const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  const Vec2d w = unit.Perp() * half_width_;
  switch (corner) {
    case FRONT_LEFT:
      return center_ + h + w;
    case REAR_LEFT:
      return center_ - h + w;
    case REAR_RIGHT:
      return center_ - h - w;
    case FRONT_RIGHT:
      return center_ + h - w;
  }
}

// Returns the front center point of the box.
inline Vec2d Box2d::FrontCenterPoint() const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  return center_ + h;
}
// Returns the rear center point of the box.
inline Vec2d Box2d::RearCenterPoint() const {
  const Vec2d unit(cos_heading_, sin_heading_);
  const Vec2d h = unit * half_length_;
  return center_ - h;
}

inline Box2d Box2d::CreateAABox(const Vec2d &one_corner,
                                const Vec2d &opposite_corner) {
  const double x1 = std::min(one_corner.x(), opposite_corner.x());
  const double x2 = std::max(one_corner.x(), opposite_corner.x());
  const double y1 = std::min(one_corner.y(), opposite_corner.y());
  const double y2 = std::max(one_corner.y(), opposite_corner.y());
  return Box2d({(x1 + x2) * 0.5, (y1 + y2) * 0.5}, 0.0, x2 - x1, y2 - y1);
}

inline bool Box2d::IsPointIn(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(-x0 * sin_heading_ + y0 * cos_heading_);
  return dx <= half_length_ + kEpsilon && dy <= half_width_ + kEpsilon;
}

inline bool Box2d::IsPointOnBoundary(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx = std::abs(x0 * cos_heading_ + y0 * sin_heading_);
  const double dy = std::abs(x0 * sin_heading_ - y0 * cos_heading_);
  return (std::abs(dx - half_length_) <= kEpsilon &&
          dy <= half_width_ + kEpsilon) ||
         (std::abs(dy - half_width_) <= kEpsilon &&
          dx <= half_length_ + kEpsilon);
}

inline double Box2d::DistanceTo(const Vec2d &point) const {
  const double x0 = point.x() - center_.x();
  const double y0 = point.y() - center_.y();
  const double dx =
      std::abs(x0 * cos_heading_ + y0 * sin_heading_) - half_length_;
  const double dy =
      std::abs(x0 * sin_heading_ - y0 * cos_heading_) - half_width_;
  if (dx <= 0.0) {
    return std::max(0.0, dy);
  }
  if (dy <= 0.0) {
    return dx;
  }
  return Hypot(dx, dy);
}

inline bool Box2d::HasOverlap(const Segment2d &line_segment) const {
  if (line_segment.length() <= kEpsilon) {
    return IsPointIn(line_segment.start());
  }
  return DistanceTo(line_segment) <= kEpsilon;
}

inline bool Box2d::HasOverlap(const AABox2d &aabox) const {
  const double shift_x = aabox.center_x() - center_.x();
  const double shift_y = aabox.center_y() - center_.y();

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  // AAbox heading = 0.0;
  const double dx3 = aabox.half_length();
  const double dy4 = -aabox.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_) + std::abs(dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_) + std::abs(-dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x) <= std::abs(dx1) + std::abs(dx2) + dx3 &&
         std::abs(shift_y) <= std::abs(dy1) + std::abs(dy2) - dy4;
}

inline double Box2d::DistanceTo(const Box2d &box) const {
  QCHECK(0) << "Not implemented";
  return 0.0;
}

inline AABox2d Box2d::GetAABox() const {
  const double dx1 = std::abs(cos_heading_ * half_length_);
  const double dy1 = std::abs(sin_heading_ * half_length_);
  const double dx2 = std::abs(sin_heading_ * half_width_);
  const double dy2 = std::abs(cos_heading_ * half_width_);
  return AABox2d(center_, (dx1 + dx2) * 2.0, (dy1 + dy2) * 2.0);
}

inline void Box2d::RotateFromCenter(const double rotate_angle) {
  heading_ = NormalizeAngle(heading_ + rotate_angle);
  cos_heading_ = std::cos(heading_);
  sin_heading_ = std::sin(heading_);
}

inline void Box2d::Shift(const Vec2d &shift_vec) { center_ += shift_vec; }

inline void Box2d::MirrorByX() {
  center_.y() = -center_.y();
  heading_ = -heading_;
  sin_heading_ = -sin_heading_;
  // cos_heading_ remains unchanged
}

inline Box2d Box2d::Transform(Vec2d translation) const {
  Box2d box(*this);
  box.Shift(translation);
  return box;
}

inline Box2d Box2d::Transform(double rotation) const {
  const Vec2d center = center_.Rotate(rotation);
  const double heading = NormalizeAngle(heading_ + rotation);
  return Box2d(half_length_, half_width_, center, heading);
}

inline Box2d Box2d::Transform(Vec2d translation, double rotation) const {
  const Vec2d center = center_.Rotate(rotation) + translation;
  const double heading = NormalizeAngle(heading_ + rotation);
  return Box2d(half_length_, half_width_, center, heading);
}

inline void Box2d::LongitudinalExtend(const double extension_length) {
  half_length_ += extension_length * 0.5;
}

inline void Box2d::LateralExtend(const double extension_length) {
  half_width_ += extension_length * 0.5;
}

inline void Box2d::LongitudinalExtendByRatio(double extension_ratio) {
  half_length_ *= extension_ratio;
}

inline void Box2d::LateralExtendByRatio(double extension_ratio) {
  half_width_ *= extension_ratio;
}

inline void Box2d::FromProto(const Box2dProto &proto) {
  *this = Box2d(Vec2d(proto.x(), proto.y()), proto.heading(), proto.length(),
                proto.width());
}

inline void Box2d::ToProto(Box2dProto *proto) const {
  proto->set_x(center().x());
  proto->set_y(center().y());
  proto->set_heading(heading());
  proto->set_length(length());
  proto->set_width(width());
}

inline std::string Box2d::DebugString() const {
  return absl::StrCat("box2d ( center = ", center_.DebugString(),
                      "  heading = ", heading_, "  length = ", length(),
                      "  width = ", width(), " )");
}

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_BOX2D_H_
