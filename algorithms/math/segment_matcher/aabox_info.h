#ifndef ONBOARD_MATH_SEGMENT_MATCHER_AABOX_INFO_H_
#define ONBOARD_MATH_SEGMENT_MATCHER_AABOX_INFO_H_

#include <string>
#include <utility>

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/segment2d.h"

namespace qcraft {

class AABoxInfo {
 public:
  AABoxInfo(Segment2d seg, std::string id);

  AABoxInfo(Segment2d seg, int index);

  virtual ~AABoxInfo() = default;

  const AABox2d& aabox() const;

  std::string id() const;

  double DistanceSquareTo(const Vec2d& point) const;

  int index() const;

  const Segment2d& segment() const;

 private:
  AABox2d aabox_;

  Segment2d segment_;

  std::string id_;

  int index_ = -1;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_SEGMENT_MATCHER_AABOX_INFO_H_
