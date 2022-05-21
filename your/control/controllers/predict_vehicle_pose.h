#ifndef ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_
#define ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_
#include <math.h>

#include <deque>

#include "onboard/control/control_defs.h"
#include "onboard/control/control_history_state_manager.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

struct VehPose {
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double heading = 0.0;
  double acc = 0.0;

  void ToProto(VehPoseProto *proto) const {
    proto->Clear();
    proto->set_x(x);
    proto->set_y(y);
    proto->set_v(v);
    proto->set_heading(heading);
  }

  void FromProto(const VehPoseProto &proto) {
    x = proto.x();
    y = proto.y();
    v = proto.v();
    heading = proto.heading();
  }
};

struct VehState {
  double steer_delay_time = 0.0;     // s.
  double throttle_delay_time = 0.0;  // s.
  VehPose init_pose;
  const ControlHistoryStateManager *control_history_state_mgr = nullptr;
};

VehPose PredictedPoseByConstAccKinematicModel(const VehState &veh_state,
                                              double wheel_base);

VehPose PredictedPoseByAccSequenceKinematicModel(const VehState &veh_state,
                                                 double wheel_base);

}  // namespace control
}  // namespace qcraft
#endif  // ONBOARD_CONTROL_CONTROLLERS_PREDICT_VEHICLE_POSE_H_
