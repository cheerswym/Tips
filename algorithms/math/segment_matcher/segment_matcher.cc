
#include "onboard/math/segment_matcher/segment_matcher.h"

#include <set>
#include "onboard/math/segment_matcher/segment_matcher_common.hpp"
#include "onboard/math/util.h"

namespace qcraft {

SegmentMatcher::SegmentMatcher(const std::vector<Vec2d>& points) {
  for (int i = 0; i + 1 < points.size(); ++i) {
    const auto& curr = points[i];
    const auto& next = points[i + 1];
    const auto seg = Segment2d({curr.x(), curr.y()}, {next.x(), next.y()});
    aa_boxes_info_.emplace_back(seg, i);
    segment_.emplace_back(seg);
  }
  index_flag_ = true;
}

SegmentMatcher::SegmentMatcher(std::vector<Segment2d> segments)
    : segment_(std::move(segments)) {
  for (int i = 0; i < segment_.size(); ++i) {
    aa_boxes_info_.emplace_back(segment_[i], i);
  }
  index_flag_ = true;
}

SegmentMatcher::SegmentMatcher(
    const std::vector<std::pair<std::string, Segment2d>>& named_segments) {
  std::set<std::string> name_set;
  segment_.reserve(named_segments.size());
  aa_boxes_info_.reserve(named_segments.size());
  for (const auto& seg : named_segments) {
    name_set.insert(seg.first);
    segment_.emplace_back(seg.second);
    aa_boxes_info_.emplace_back(seg.second, seg.first);
    segment_map_.insert(std::make_pair(seg.first, &segment_.back()));
  }
  QCHECK_EQ(name_set.size(), named_segments.size())
      << " segments id duplicate ";
  named_flag_ = true;
}

const Segment2d* SegmentMatcher::GetSegmentByIndex(int index) const {
  if (index + 1 > segment_.size()) {
    return nullptr;
  }
  return &segment_[index];
}

const Segment2d* SegmentMatcher::GetSegmentById(const std::string& id) const {
  const auto& it = segment_map_.find(id);
  return it != segment_map_.end() ? it->second : nullptr;
}

bool SegmentMatcher::GetProjection(double x, double y, bool is_clamp,
                                   double* const accumulated_s,
                                   double* const lateral) const {
  QCHECK(index_flag_);
  QCHECK_NOTNULL(accumulated_s);
  QCHECK_NOTNULL(lateral);
  int idx = 0;
  if (!GetNearestSegmentIndex(x, y, &idx)) {
    return false;
  }
  const auto& nearest_seg = segment_[idx];
  double length = 0.0;
  for (int i = 0; i < idx; ++i) {
    length += segment_[i].length();
  }
  double proj = nearest_seg.ProjectOntoUnit({x, y});
  double prod = nearest_seg.ProductOntoUnit({x, y});
  proj = is_clamp ? std::clamp<double>(proj, 0, nearest_seg.length()) : proj;

  *accumulated_s = length + proj;
  *lateral = prod;
  return true;
}

bool SegmentMatcher::GetProjection(double x, double y,
                                   Vec2d* const nearest_point,
                                   double* const accumulated_s,
                                   double* const min_dist,
                                   Segment2d* const segment) const {
  QCHECK(index_flag_);
  int idx = 0;
  if (!GetNearestSegmentIndex(x, y, &idx)) return false;
  const auto& nearest_seg = segment_[idx];
  const double dist = nearest_seg.DistanceTo({x, y}, nearest_point);
  if (min_dist != nullptr) *min_dist = dist;
  if (accumulated_s != nullptr) {
    double length = 0.0;
    for (int i = 0; i < idx; ++i) {
      length += segment_[i].length();
    }
    *accumulated_s = length + nearest_seg.ProjectOntoUnit({x, y});
  }
  if (segment != nullptr) *segment = segment_.at(idx);
  return true;
}

bool SegmentMatcher::GetNearestSegmentIndexWithHeading(
    double x, double y, double theta, double max_radius,
    double max_heading_diff, int* const nearest_index) const {
  QCHECK(index_flag_);
  QCHECK_NOTNULL(nearest_index);
  std::vector<AABoxInfo> partial_aaboxes;
  for (const auto& index : GetSegmentIndexInRadius(x, y, max_radius)) {
    const double heading_diff =
        NormalizeAngle(theta - segment_[index].heading());
    if (std::abs(heading_diff) > max_heading_diff) {
      continue;
    }
    partial_aaboxes.emplace_back(segment_[index], index);
  }
  int index_in_partial = 0;
  if (!NearestSegsIndex(partial_aaboxes, x, y, &index_in_partial)) {
    return false;
  }
  *nearest_index = partial_aaboxes[index_in_partial].index();
  return true;
}

bool SegmentMatcher::GetNearestSegmentIdWithHeading(
    double x, double y, double theta, double max_radius,
    double max_heading_diff, std::string* const nearest_id) const {
  QCHECK(named_flag_);
  QCHECK_NOTNULL(nearest_id);
  std::vector<AABoxInfo> partial_aaboxes;
  for (const auto& id : GetSegmentIdInRadius(x, y, max_radius)) {
    const double heading_diff =
        NormalizeAngle(theta - GetSegmentById(id)->heading());
    if (std::abs(heading_diff) > max_heading_diff) {
      continue;
    }
    partial_aaboxes.emplace_back(*GetSegmentById(id), id);
  }
  int index_in_partial = 0;
  if (!NearestSegsIndex(partial_aaboxes, x, y, &index_in_partial)) {
    return false;
  }
  *nearest_id = partial_aaboxes[index_in_partial].id();
  return true;
}

std::vector<std::string> SegmentMatcher::GetSegmentIdInRadiusWithHeading(
    double x, double y, double theta, double max_radius,
    double max_heading_diff) const {
  QCHECK(named_flag_);
  std::vector<std::string> ids_with_in_radius;
  for (const auto& id : GetSegmentIdInRadius(x, y, max_radius)) {
    const double heading_diff =
        NormalizeAngle(theta - GetSegmentById(id)->heading());
    if (std::fabs(heading_diff) > max_heading_diff) {
      continue;
    }
    ids_with_in_radius.emplace_back(id);
  }
  return ids_with_in_radius;
}

}  // namespace qcraft
