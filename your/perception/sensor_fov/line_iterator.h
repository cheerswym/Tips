#ifndef ONBOARD_PERCEPTION_SENSOR_FOV_LINE_ITERATOR_H_
#define ONBOARD_PERCEPTION_SENSOR_FOV_LINE_ITERATOR_H_

#include <utility>
#include <vector>

#include "onboard/math/vec.h"

namespace qcraft::sensor_fov {

class LineIterator {
 public:
  // Creates iterators for the line connecting pt1 and pt2
  // the line will be clipped on the image boundaries
  // the line is 8-connected or 4-connected
  LineIterator(const Vec2i pt1, const Vec2i pt2, const int connectivity = 8)
      : count_(-1),
        err_(0),
        minus_delta_(0),
        plus_delta_(0),
        minus_step_(0),
        plus_step_(0),
        minus_shift_(0),
        plus_shift_(0),
        point_() {
    Init(pt1, pt2, connectivity);
  }

  // Coordinates of the current pixel
  Vec2i operator*() const { return pos(); }
  // Prefix increment operator (++it). shifts iterator to the next pixel
  LineIterator& operator++() {
    const int mask = err_ < 0 ? -1 : 0;
    err_ += minus_delta_ + (plus_delta_ & mask);
    point_.x() += minus_shift_ + (plus_shift_ & mask);
    point_.y() += minus_step_ + (plus_step_ & mask);
    return *this;
  }
  // Postfix increment operator (it++). shifts iterator to the next
  const LineIterator operator++(int) {
    LineIterator it = *this;
    ++(*this);
    return it;
  }
  // Coordinates of the current pixel
  Vec2i pos() const { return point_; }
  // Total num pixels.
  int count() const { return count_; }

 private:
  void Init(const Vec2i pt1, const Vec2i pt2, int connectivity);

 private:
  int count_;
  int err_;
  int minus_delta_;
  int plus_delta_;
  int minus_step_;
  int plus_step_;
  int minus_shift_;
  int plus_shift_;
  Vec2i point_;
};

inline std::vector<Vec2i> ComputeAllLinePoints(const Vec2i pt1, const Vec2i pt2,
                                               const int connectivity = 8) {
  LineIterator li(pt1, pt2, connectivity);
  std::vector<Vec2i> points;
  points.reserve(li.count());
  for (int i = 0; i < li.count(); ++i, ++li) {
    points.emplace_back(li.pos());
  }
  return points;
}

}  // namespace qcraft::sensor_fov

#endif  // ONBOARD_PERCEPTION_SENSOR_FOV_LINE_ITERATOR_H_
