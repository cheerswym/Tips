#include "onboard/control/controllers/predict_vehicle_pose.h"

#include <algorithm>

#include "onboard/lite/logging.h"

namespace qcraft {
namespace control {

VehPose PredictedPoseByConstAccKinematicModel(const VehState &veh_state,
                                              double wheel_base) {
  VehPose veh_pose;
  veh_pose = veh_state.init_pose;

  const int steer_delay_cycle =
      FloorToInt(veh_state.steer_delay_time / kControlInterval);
  const int control_state_cache_size =
      veh_state.control_history_state_mgr->GetControlStateCache().size();

  if (control_state_cache_size <= steer_delay_cycle) {
    QLOG_EVERY_N_SEC(ERROR, 1) << "[Control] Cache data is not ready.";
    return veh_pose;
  }

  for (int i = 0; i < steer_delay_cycle; ++i) {
    veh_pose.v = veh_pose.v + veh_pose.acc * kControlInterval;
    if (veh_state.init_pose.v * veh_pose.v <=
        0.0) {  // Vehicle stopped or go to the opposite direction.
      veh_pose.v = 0.0;
      return veh_pose;
    }
    veh_pose.x +=
        veh_pose.v * fast_math::Cos(veh_pose.heading) * kControlInterval;
    veh_pose.y +=
        veh_pose.v * fast_math::Sin(veh_pose.heading) * kControlInterval;

    const double kappa_target_wrt_delay =
        veh_state.control_history_state_mgr
            ->GetControlStateCache()[control_state_cache_size -
                                     steer_delay_cycle + i]
            .kappa_cmd;

    veh_pose.heading =
        NormalizeAngle(veh_pose.heading +
                       veh_pose.v * kappa_target_wrt_delay * kControlInterval);
  }
  return veh_pose;
}

VehPose PredictedPoseByAccSequenceKinematicModel(const VehState &veh_state,
                                                 double wheel_base) {
  VehPose veh_pose;
  veh_pose = veh_state.init_pose;

  const int steer_delay_cycle =
      FloorToInt(veh_state.steer_delay_time / kControlInterval);
  // To forbid switching between throttle and brake, always use throttle delay
  // for which most of time, throttle is used rather than brake.
  const int throttle_delay_cycle =
      std::max(FloorToInt(veh_state.throttle_delay_time / kControlInterval),
               steer_delay_cycle);
  const int control_state_cache_size =
      veh_state.control_history_state_mgr->GetControlStateCache().size();

  if (control_state_cache_size <= steer_delay_cycle) {
    QLOG_EVERY_N_SEC(ERROR, 1) << "[Control] Cache data is not ready.";
    return veh_pose;
  }

  // TODO(shijun): deal with throttle delay time < steer delay time.
  for (int i = 0; i < steer_delay_cycle; ++i) {
    const double acc_target_wrt_delay =
        veh_state.control_history_state_mgr
            ->GetControlStateCache()[control_state_cache_size -
                                     throttle_delay_cycle + i]
            .acc_target;
    veh_pose.v += acc_target_wrt_delay * kControlInterval;
    if (veh_state.init_pose.v * veh_pose.v <=
        0.0) {  // Vehicle stopped or go to the opposite direction.
      veh_pose.v = 0.0;
      return veh_pose;
    }
    veh_pose.x +=
        veh_pose.v * fast_math::Cos(veh_pose.heading) * kControlInterval;
    veh_pose.y +=
        veh_pose.v * fast_math::Sin(veh_pose.heading) * kControlInterval;

    const double kappa_cmd_wrt_dellay =
        veh_state.control_history_state_mgr
            ->GetControlStateCache()[control_state_cache_size -
                                     steer_delay_cycle + i]
            .kappa_cmd;

    veh_pose.heading =
        NormalizeAngle(veh_pose.heading +
                       veh_pose.v * kappa_cmd_wrt_dellay * kControlInterval);
  }
  return veh_pose;
}

}  // namespace control
}  // namespace qcraft
