#include "onboard/math/frenet_frame.h"

#include <algorithm>
#include <limits>

#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {

Vec2d FrenetFrame::InterpolateTangentByS(double s) const {
  const auto it = std::lower_bound(s_knots_.begin(), s_knots_.end(), s);
  if (it == s_knots_.begin()) {
    return tangents_.front();
  }
  if (it == s_knots_.end() || it == s_knots_.end() - 1) {
    return tangents_.back();
  }
  if (std::fabs(s - *it) < 1e-3) {
    double t = (s - *(it - 1)) / (*(it + 1) - *(it - 1));
    return Vec2d::FastUnitFromAngle(
        Lerp(tangents_[it - 1 - s_knots_.begin()].Angle(),
             tangents_[it - s_knots_.begin()].Angle(), t));
  }
  return tangents_[it - 1 - s_knots_.begin()];
}

Vec2d FrenetFrame::InterpolateTangentByXY(const Vec2d &xy) const {
  Vec2d sl;
  Vec2d normal;
  int index;
  double alpha = 0.0;
  XYToSL(xy, &sl, &normal, &index, &alpha);
  alpha = std::clamp(alpha - 1, 0.0, 1.0);
  return Vec2d::FastUnitFromAngle(
      Lerp(tangents_[index].Angle(), tangents_[index + 1].Angle(), alpha));
}

Vec2d FrenetFrame::SLToXY(const Vec2d &sl) const {
  Vec2d prev_pt, succ_pt;
  double interp_t;
  std::tie(prev_pt, succ_pt, interp_t) = GetInterpolationRange(sl.x());
  const Vec2d heading_vec = (succ_pt - prev_pt).normalized();
  const Vec2d normal_vec = heading_vec.Perp();

  return Lerp(prev_pt, succ_pt, interp_t) + normal_vec * sl.y();
}

Vec2d FrenetFrame::XYToSL(const Vec2d &xy) const {
  Vec2d sl;
  Vec2d normal;
  XYToSL(xy, &sl, &normal);
  return sl;
}

void FrenetFrame::XYToSL(const Vec2d &xy, Vec2d *sl, Vec2d *normal) const {
  int index;
  double alpha = 0.0;
  XYToSL(xy, sl, normal, &index, &alpha);
}

absl::StatusOr<Vec2d> FrenetFrame::XYToSLWithHeadingDiffLimit(
    const Vec2d &xy, double heading, double max_heading_diff) const {
  Vec2d sl;
  Vec2d normal;
  int index = 0;
  double alpha = 0.0;

  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 1; i < points_.size(); ++i) {
    const Vec2d p0 = points_[i - 1];
    const Vec2d p1 = points_[i];
    const Vec2d heading_vec = tangents_[i - 1];
    const Vec2d query_segment = xy - p0;
    const double projection =
        query_segment.dot(heading_vec) * segment_len_inv_[i - 1];
    const double production = heading_vec.CrossProd(query_segment);
    double l = std::numeric_limits<double>::infinity();
    double s = 0.0;
    if (projection < 0.0 && i > 1) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p0.DistanceTo(xy) * sign;
      s = s_knots_[i - 1];
    } else if (projection > 1.0 && i + 1 < points_.size()) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p1.DistanceTo(xy) * sign;
      s = s_knots_[i];
    } else {
      l = production;
      s = Lerp(s_knots_[i - 1], s_knots_[i], projection);
    }

    if (std::fabs(NormalizeAngle(heading - heading_vec.FastAngle())) >
        max_heading_diff) {
      continue;
    }

    if (std::fabs(l) < min_d) {
      min_d = std::fabs(l);
      sl = Vec2d(s, l);
      normal = heading_vec.Perp();
      index = i - 1;
      alpha = projection;
    }
  }
  if (min_d < std::numeric_limits<double>::infinity()) {
    return sl;
  }
  return absl::NotFoundError("No viable frenet projection!");
}

void FrenetFrame::XYToSL(const Vec2d &xy, Vec2d *sl, Vec2d *normal, int *index,
                         double *alpha) const {
  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 1; i < points_.size(); ++i) {
    const Vec2d p0 = points_[i - 1];
    const Vec2d p1 = points_[i];
    const Vec2d heading_vec = tangents_[i - 1];
    const Vec2d query_segment = xy - p0;
    const double projection =
        query_segment.dot(heading_vec) * segment_len_inv_[i - 1];
    const double production = heading_vec.CrossProd(query_segment);
    double l = 0.0;
    double s = 0.0;
    if (projection < 0.0 && i > 1) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p0.DistanceTo(xy) * sign;
      s = s_knots_[i - 1];
    } else if (projection > 1.0 && i + 1 < points_.size()) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p1.DistanceTo(xy) * sign;
      s = s_knots_[i];
    } else {
      l = production;
      s = Lerp(s_knots_[i - 1], s_knots_[i], projection);
    }

    if (std::fabs(l) < min_d) {
      min_d = std::fabs(l);
      *sl = Vec2d(s, l);
      *normal = heading_vec.Perp();
      *index = i - 1;
      *alpha = projection;
    }
  }
}

void FrenetFrame::XYToSL(const Vec2d &xy, Vec2d *sl, Vec2d *normal,
                         std::pair<int, int> *raw_index_pair,
                         double *alpha) const {
  double min_d = std::numeric_limits<double>::infinity();
  for (int i = 1; i < points_.size(); ++i) {
    const Vec2d p0 = points_[i - 1];
    const Vec2d p1 = points_[i];
    const Vec2d heading_vec = tangents_[i - 1];
    const Vec2d query_segment = xy - p0;
    const double projection =
        query_segment.dot(heading_vec) * segment_len_inv_[i - 1];
    const double production = heading_vec.CrossProd(query_segment);
    double l = 0.0;
    double s = 0.0;
    if (projection < 0.0 && i > 1) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p0.DistanceTo(xy) * sign;
      s = s_knots_[i - 1];
    } else if (projection > 1.0 && i + 1 < points_.size()) {
      const double sign = production < 0.0 ? -1.0 : 1.0;
      l = p1.DistanceTo(xy) * sign;
      s = s_knots_[i];
    } else {
      l = production;
      s = Lerp(s_knots_[i - 1], s_knots_[i], projection);
    }

    if (std::fabs(l) < min_d) {
      min_d = std::fabs(l);
      *sl = Vec2d(s, l);
      *normal = heading_vec.Perp();
      raw_index_pair->first = raw_indices_[i - 1];
      raw_index_pair->second = raw_indices_[i];
      *alpha = projection;
    }
  }
}

std::vector<Vec2d> FrenetFrame::XYToSL(absl::Span<const Vec2d> xy) const {
  std::vector<Vec2d> sl(xy.size());
  std::vector<Vec2d> normal(xy.size());
  XYToSL(xy, absl::MakeSpan(sl), absl::MakeSpan(normal));
  return sl;
}

void FrenetFrame::XYToSL(absl::Span<const Vec2d> xy, absl::Span<Vec2d> sl,
                         absl::Span<Vec2d> normal) const {
  std::vector<std::pair<int, int>> index_pairs(xy.size());
  std::vector<double> alpha(xy.size());
  XYToSL(xy, sl, normal, absl::MakeSpan(index_pairs), absl::MakeSpan(alpha));
}

void FrenetFrame::XYToSL(absl::Span<const Vec2d> xy, absl::Span<Vec2d> sl,
                         absl::Span<Vec2d> normal,
                         absl::Span<std::pair<int, int>> index_pairs,
                         absl::Span<double> alpha) const {
  // Sanity checks.
  QCHECK_GT(xy.size(), 1);
  QCHECK_EQ(xy.size(), sl.size());
  QCHECK_EQ(xy.size(), normal.size());
  QCHECK_EQ(xy.size(), index_pairs.size());
  QCHECK_EQ(xy.size(), alpha.size());

  int index = 1;
  for (int i = 0; i < xy.size(); ++i) {
    for (int j = index; j < points_.size(); ++j) {
      const Vec2d p0 = points_[j - 1];
      const Vec2d p1 = points_[j];
      const Vec2d heading_vec = tangents_[j - 1];
      const Vec2d query_segment = xy[i] - p0;
      const double projection =
          query_segment.dot(heading_vec) * segment_len_inv_[j - 1];
      const double production = heading_vec.CrossProd(query_segment);
      double l = 0.0;
      double s = 0.0;
      if (projection < 0.0 && j > 1) {
        const double sign = production < 0.0 ? -1.0 : 1.0;
        l = p0.DistanceTo(xy[i]) * sign;
        s = s_knots_[j - 1];
      } else if (projection > 1.0 && j + 1 < points_.size()) {
        const double sign = production < 0.0 ? -1.0 : 1.0;
        l = p1.DistanceTo(xy[i]) * sign;
        s = s_knots_[j];
      } else {
        l = production;
        s = Lerp(s_knots_[j - 1], s_knots_[j], projection);
      }

      // Fast exit is enabled here, so l is not guaranteed to be the minimal
      // distance between the point and the frenet line
      if ((xy[i] - p1).dot(heading_vec) < 0.0 || j == points_.size() - 1) {
        sl[i] = Vec2d(s, l);
        normal[i] = heading_vec.Perp();
        index_pairs[i].first = raw_indices_[j - 1];
        index_pairs[i].second = raw_indices_[j];
        alpha[i] = projection;
        index = j;
        break;
      }
    }
  }
}

void FrenetFrame::Build(const std::vector<Vec2d> &raw_points) {
  QCHECK_GE(raw_points.size(), 2);
  points_.clear();
  s_knots_.clear();
  tangents_.clear();
  raw_indices_.clear();

  points_.reserve(raw_points.size());
  raw_indices_.reserve(raw_points.size());

  points_.emplace_back(raw_points.front());
  raw_indices_.push_back(0);

  constexpr double kMinSampleDistanceSqr = Sqr(0.1);  // m^2.
  for (int i = 1; i < raw_points.size(); ++i) {
    const double distance_sqr = raw_points[i].DistanceSquareTo(points_.back());
    if (distance_sqr < kMinSampleDistanceSqr) {
      if (i + 1 != raw_points.size()) continue;
      // Push back last point, delete previous one if there are more than one
      // points.
      if (i > 1) {
        points_.erase(points_.end() - 1);
        raw_indices_.erase(raw_indices_.end() - 1);
      }
    }
    points_.emplace_back(raw_points[i]);
    raw_indices_.push_back(i);
  }
  QCHECK_EQ(points_.size(), raw_indices_.size());
  QCHECK_GE(points_.size(), 2)
      << " raw points size " << raw_points.size() << " raw point front ("
      << raw_points.front().x() << ", " << raw_points.front().y()
      << ") raw point back (" << raw_points.back().x() << ", "
      << raw_points.back().y() << ")";

  s_knots_.reserve(points_.size());
  s_knots_.emplace_back(0.0);
  segment_len_inv_.reserve(points_.size() - 1);
  tangents_.reserve(points_.size());
  for (int i = 1; i < points_.size(); ++i) {
    const Vec2d segment = points_[i] - points_[i - 1];
    const double segment_len = segment.norm();
    const double segment_len_inv = 1.0 / segment_len;
    s_knots_.emplace_back(s_knots_.back() + segment_len);
    segment_len_inv_.emplace_back(segment_len_inv);
    tangents_.emplace_back(segment * segment_len_inv);
  }
  tangents_.emplace_back(tangents_.back());
}

std::tuple<Vec2d, Vec2d, double> FrenetFrame::GetInterpolationRange(
    double s) const {
  // Allow extra-polation
  const auto it = std::lower_bound(s_knots_.begin(), s_knots_.end(), s);

  if (it == s_knots_.begin()) {
    return {points_[0], points_[1], (s - *it) / (*(it + 1) - *it)};
  }

  if (it == s_knots_.end()) {
    return {points_[points_.size() - 2], points_.back(),
            (s - *(it - 2)) / (*(it - 1) - *(it - 2))};
  }

  return {points_[it - 1 - s_knots_.begin()], points_[it - s_knots_.begin()],
          (s - *(it - 1)) / (*it - *(it - 1))};
}

Vec2d FrenetFrame::FindAABBNearestPoint(const Polygon2d &polygon,
                                        bool use_max_s) const {
  double smax = -std::numeric_limits<double>::infinity();
  double smin = std::numeric_limits<double>::infinity();
  double lmax = -std::numeric_limits<double>::infinity();
  double lmin = std::numeric_limits<double>::infinity();

  for (const auto &pt : polygon.points()) {
    Vec2d sl = XYToSL(pt);
    smax = std::max(smax, sl.x());
    smin = std::min(smin, sl.x());
    lmax = std::max(lmax, sl.y());
    lmin = std::min(lmin, sl.y());
  }

  double near_l;
  if (lmax * lmin < 0) {
    near_l = 0.0;
  } else {
    near_l = std::abs(lmax) > std::abs(lmin) ? lmin : lmax;
  }

  return Vec2d(use_max_s ? smax : smin, near_l);
}

}  // namespace qcraft
