#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_STATE_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_STATE_H_

#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"

namespace qcraft::planner {

// This struct represents a vehicle's state in spacetime.
struct MotionState {
  Vec2d xy;
  double h = 0.0;              // heading.
  double k = 0.0;              // curvature.
  double ref_k = 0.0;          // reference curvature.
  double t = 0.0;              // time.
  double v = 0.0;              // velocity.
  double a = 0.0;              // acceleration.
  double accumulated_s = 0.0;  // accumulated s along the drive passage.
  double s = 0.0;              // arc length of the trajectory.
  double l = 0.0;              // lateral offset.
  std::string DebugString() const {
    return absl::StrCat("xy:", xy.DebugString(), ", h:", h, ", k:", k,
                        ", t:", t, ", v:", v, ", a:", a,
                        ", accumulated_s:", accumulated_s, ", arc_s:", s);
  }
  void ToProto(MotionStateProto* proto) const {
    proto->set_x(xy.x());
    proto->set_y(xy.y());
    proto->set_k(k);
    proto->set_h(h);
    proto->set_t(t);
    proto->set_v(v);
    proto->set_a(a);
  }
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_STATE_H_
