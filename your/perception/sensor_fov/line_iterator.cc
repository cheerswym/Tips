#include "onboard/perception/sensor_fov/line_iterator.h"

#include <algorithm>

#include "onboard/lite/logging.h"

namespace qcraft::sensor_fov {

void LineIterator::Init(const Vec2i pt1, const Vec2i pt2,
                        const int connectivity) {
  QCHECK(connectivity == 8 || connectivity == 4);

  int delta_x = 1, delta_y = 1;
  int dx = pt2.x() - pt1.x();
  int dy = pt2.y() - pt1.y();

  if (dx < 0) {
    dx = -dx;
    delta_x = -1;
  }

  if (dy < 0) {
    dy = -dy;
    delta_y = -1;
  }

  const bool vert = dy > dx;
  if (vert) {
    std::swap(dx, dy);
    std::swap(delta_x, delta_y);
  }

  QCHECK(dx >= 0 && dy >= 0);

  if (connectivity == 8) {
    err_ = dx - (dy + dy);
    plus_delta_ = dx + dx;
    minus_delta_ = -(dy + dy);
    minus_shift_ = delta_x;
    plus_shift_ = 0;
    minus_step_ = 0;
    plus_step_ = delta_y;
    count_ = dx + 1;
  } else { /* connectivity == 4 */
    err_ = 0;
    plus_delta_ = (dx + dx) + (dy + dy);
    minus_delta_ = -(dy + dy);
    minus_shift_ = delta_x;
    plus_shift_ = -delta_x;
    minus_step_ = 0;
    plus_step_ = delta_y;
    count_ = dx + dy + 1;
  }

  if (vert) {
    std::swap(plus_step_, plus_shift_);
    std::swap(minus_step_, minus_shift_);
  }

  point_ = pt1;
}

}  // namespace qcraft::sensor_fov
