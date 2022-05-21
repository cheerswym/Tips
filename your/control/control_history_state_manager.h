#ifndef ONBOARD_CONTROL_CONTROL_HISTORY_STATE_MANAGER_H_
#define ONBOARD_CONTROL_CONTROL_HISTORY_STATE_MANAGER_H_

#include "boost/circular_buffer.hpp"

namespace qcraft {
namespace control {
static constexpr int kCacheSize = 200;  // 2s.
class ControlHistoryStateManager {
 public:
  ControlHistoryStateManager();
  struct ControlStateData {
    bool is_under_control = false;
    double kappa_cmd = 0.0;          // control_command.curvature();
    double steer_pct_chassis = 0.0;  // chassis.steering_percentage();

    // Refer to the doc of Post-processing of longitudinal control -
    // acceleration signal definitions".
    double acc_target = 0.0;
  };

  void UpdateHistoryData(bool is_under_control, double kappa_cmd,
                         double steer_pct_chassis, double acc_target);

  const boost::circular_buffer<ControlStateData>& GetControlStateCache() const {
    return control_state_cache_;
  }

 private:
  boost::circular_buffer<ControlStateData> control_state_cache_{kCacheSize};
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROL_HISTORY_STATE_MANAGER_H_
