#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"

namespace qcraft {

SegmentMatcherKdtree::SegmentMatcherKdtree(std::vector<Vec2d> points)
    : SegmentMatcher(std::move(points)) {
  InitAAboxKdtree();
}

SegmentMatcherKdtree::SegmentMatcherKdtree(std::vector<Segment2d> segments)
    : SegmentMatcher(std::move(segments)) {
  InitAAboxKdtree();
}

SegmentMatcherKdtree::SegmentMatcherKdtree(
    std::vector<std::pair<std::string, Segment2d>> named_segments)
    : SegmentMatcher(std::move(named_segments)) {
  InitAAboxKdtree();
}

void SegmentMatcherKdtree::InitAAboxKdtree() {
  AABoxKDTreeParams params;
  segments_tree_ =
      std::make_unique<AABoxKDTree2d<AABoxInfo>>(aa_boxes_info_, params);
}

bool SegmentMatcherKdtree::GetNearestSegmentId(double x, double y,
                                               std::string* const id) const {
  QCHECK(named_flag_) << " construct without id";
  QCHECK_NOTNULL(id);
  const auto& object = segments_tree_->GetNearestObject({x, y});
  if (object == nullptr) {
    return false;
  }
  *id = object->id();
  return true;
}

std::vector<std::string> SegmentMatcherKdtree::GetSegmentIdInRadius(
    double x, double y, double r) const {
  QCHECK(named_flag_) << " construct without id";
  const auto& aabox_in_radius = segments_tree_->GetObjects({x, y}, r);
  std::vector<std::string> names;
  names.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    names.emplace_back(aabox->id());
  }
  return names;
}

bool SegmentMatcherKdtree::GetNearestSegmentIndex(double x, double y,
                                                  int* const index) const {
  QCHECK(index_flag_) << " construct without index";
  QCHECK_NOTNULL(index);
  const auto& object = segments_tree_->GetNearestObject({x, y});
  if (object == nullptr) {
    return false;
  }
  *index = object->index();
  return true;
}

std::vector<int> SegmentMatcherKdtree::GetSegmentIndexInRadius(double x,
                                                               double y,
                                                               double r) const {
  QCHECK(index_flag_) << " construct without index";
  const auto& aabox_in_radius = segments_tree_->GetObjects({x, y}, r);
  std::vector<int> indexes;
  indexes.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    indexes.emplace_back(aabox->index());
  }
  return indexes;
}

const Segment2d* SegmentMatcherKdtree::GetNearestSegment(double x,
                                                         double y) const {
  const auto& object = segments_tree_->GetNearestObject({x, y});
  if (object == nullptr) {
    return nullptr;
  }
  return &object->segment();
}

std::vector<const Segment2d*> SegmentMatcherKdtree::GetSegmentInRadius(
    double x, double y, double r) const {
  const auto& aabox_in_radius = segments_tree_->GetObjects({x, y}, r);
  std::vector<const Segment2d*> segments_ptr;
  segments_ptr.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    segments_ptr.emplace_back(&aabox->segment());
  }
  return segments_ptr;
}

bool SegmentMatcherKdtree::GetNearestNamedSegment(double x, double y,
                                                  Segment2d* const seg,
                                                  std::string* const id) const {
  QCHECK(named_flag_) << " construct without id";
  QCHECK_NOTNULL(seg);
  QCHECK_NOTNULL(id);
  const auto& object = segments_tree_->GetNearestObject({x, y});
  if (object == nullptr) {
    return false;
  }
  *seg = object->segment();
  *id = object->id();
  return true;
}

std::vector<std::pair<const Segment2d*, std::string>>
SegmentMatcherKdtree::GetNamedSegmentsInRadius(double x, double y,
                                               double r) const {
  QCHECK(named_flag_) << " construct without id";
  const auto& aabox_in_radius = segments_tree_->GetObjects({x, y}, r);
  std::vector<std::pair<const Segment2d*, std::string>> named_segments;
  named_segments.reserve(aabox_in_radius.size());
  for (const auto& aabox : aabox_in_radius) {
    named_segments.emplace_back(&aabox->segment(), aabox->id());
  }
  return named_segments;
}

}  // namespace qcraft
