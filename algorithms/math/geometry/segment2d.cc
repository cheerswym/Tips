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

#include "onboard/math/geometry/segment2d.h"

#include <algorithm>
#include <utility>

#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {

double Segment2d::DistanceTo(const Vec2d &point,
                             Vec2d *const nearest_pt) const {
  if (length_ <= kEpsilon) {
    if (nearest_pt != nullptr) *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    if (nearest_pt != nullptr) *nearest_pt = start_;
    return Hypot(x0, y0);
  }
  if (proj > length_) {
    if (nearest_pt != nullptr) *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  if (nearest_pt != nullptr) *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::DistanceTo(const Segment2d &segment) const {
  Vec2d nearest_pt, other_nearest_pt;
  return DistanceTo(segment, &nearest_pt, &other_nearest_pt);
}

double Segment2d::DistanceTo(const Segment2d &segment, Vec2d *nearest_pt,
                             Vec2d *other_nearest_pt) const {
  Vec2d intersection_point;
  if (GetIntersect(segment, &intersection_point)) {
    if (nearest_pt != nullptr) *nearest_pt = intersection_point;
    if (other_nearest_pt != nullptr) *other_nearest_pt = intersection_point;
    return 0.0;
  }

  Vec2d nearest_pt_start, nearest_pt_end;
  const double dist_start = DistanceTo(segment.start(), &nearest_pt_start);
  const double dist_end = DistanceTo(segment.end(), &nearest_pt_end);

  Vec2d other_nearest_pt_start, other_nearest_pt_end;
  const double other_dist_start =
      segment.DistanceTo(start(), &other_nearest_pt_start);
  const double other_dist_end =
      segment.DistanceTo(end(), &other_nearest_pt_end);

  const double min_dist = std::min(std::min(dist_start, dist_end),
                                   std::min(other_dist_start, other_dist_end));
  if (min_dist == dist_start) {
    if (nearest_pt != nullptr) *nearest_pt = nearest_pt_start;
    if (other_nearest_pt != nullptr) *other_nearest_pt = segment.start();
  } else if (min_dist == dist_end) {
    if (nearest_pt != nullptr) *nearest_pt = nearest_pt_end;
    if (other_nearest_pt != nullptr) *other_nearest_pt = segment.end();
  } else if (min_dist == other_dist_start) {
    if (nearest_pt != nullptr) *nearest_pt = start();
    if (other_nearest_pt != nullptr) *other_nearest_pt = other_nearest_pt_start;
  } else if (min_dist == other_dist_end) {
    if (nearest_pt != nullptr) *nearest_pt = end();
    if (other_nearest_pt != nullptr) *other_nearest_pt = other_nearest_pt_end;
  }
  return min_dist;
}

double Segment2d::DistanceSquareTo(const Vec2d &point) const {
  if (length_ <= kEpsilon) {
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return Sqr(x0) + Sqr(y0);
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return Sqr(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::DistanceSquareTo(const Vec2d &point,
                                   Vec2d *const nearest_pt) const {
  QCHECK_NOTNULL(nearest_pt);
  if (length_ <= kEpsilon) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return Sqr(x0) + Sqr(y0);
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return Sqr(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

double Segment2d::ProjectOntoUnit(const Vec2d &point) const {
  return unit_direction_.dot(point - start_);
}

double Segment2d::ProductOntoUnit(const Vec2d &point) const {
  return unit_direction_.CrossProd(point - start_);
}

bool Segment2d::HasIntersect(const Segment2d &other_segment) const {
  Vec2d point;
  return GetIntersect(other_segment, &point);
}

bool Segment2d::GetIntersect(const Segment2d &other_segment,
                             Vec2d *const point) const {
  QCHECK_NOTNULL(point);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kEpsilon || other_segment.length() <= kEpsilon) {
    return false;
  }
  const double cc1 = CrossProd(start_, end_, other_segment.start());
  const double cc2 = CrossProd(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kEpsilon) {
    return false;
  }
  const double cc3 =
      CrossProd(other_segment.start(), other_segment.end(), start_);
  const double cc4 =
      CrossProd(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kEpsilon) {
    return false;
  }
  const double ratio = cc4 / (cc4 - cc3);
  *point = Vec2d(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
double Segment2d::GetPerpendicularFoot(const Vec2d &point,
                                       Vec2d *const foot_point) const {
  QCHECK_NOTNULL(foot_point);
  if (length_ <= kEpsilon) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const double x0 = point.x() - start_.x();
  const double y0 = point.y() - start_.y();
  const double proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

std::string Segment2d::DebugString() const {
  return absl::StrCat("segment2d ( start = ", start_.DebugString(),
                      "  end = ", end_.DebugString(), " )");
}

}  // namespace qcraft
