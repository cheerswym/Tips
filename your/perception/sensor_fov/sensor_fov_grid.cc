#include "onboard/perception/sensor_fov/sensor_fov_grid.h"

namespace qcraft::sensor_fov {

namespace {}  // namespace

DetectionRangeProto SFDetectionRange::ToProto() const {
  DetectionRangeProto proto;
  proto.set_front(front);
  proto.set_behind(behind);
  proto.set_left(left);
  proto.set_right(right);
  return proto;
}

void SFDetectionRange::FromProto(const DetectionRangeProto& proto) {
  front = proto.front();
  behind = proto.behind();
  left = proto.left();
  right = proto.right();
}

Box2d SFDetectionRange::ToBox2d() const {
  return Box2d(Vec2d((front - behind) * 0.5, (left - right) * 0.5), 0,
               front + behind, left + right);
}

}  // namespace qcraft::sensor_fov
