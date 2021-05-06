#include "onboard/math/segment_matcher/aabox_info.h"

#include "onboard/lite/check.h"

namespace qcraft {

AABoxInfo::AABoxInfo(Segment2d seg, std::string id)
    : aabox_(seg.start(), seg.end()),
      segment_(std::move(seg)),
      id_(std::move(id)) {
  QCHECK((segment_.start() != segment_.end()))
      << segment_.DebugString() << " id:  " << id_;
}

AABoxInfo::AABoxInfo(Segment2d seg, int index)
    : aabox_(seg.start(), seg.end()), segment_(std::move(seg)), index_(index) {}

const AABox2d& AABoxInfo::aabox() const { return aabox_; }

double AABoxInfo::DistanceSquareTo(const Vec2d& point) const {
  return segment_.DistanceSquareTo(point);
}

std::string AABoxInfo::id() const { return id_; }

int AABoxInfo::index() const { return index_; }

const Segment2d& AABoxInfo::segment() const { return segment_; }

}  // namespace qcraft
