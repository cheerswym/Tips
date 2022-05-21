#include "onboard/control/control_history_state_manager.h"

namespace qcraft {
namespace control {

ControlHistoryStateManager::ControlHistoryStateManager() {
  ControlStateData default_control_state_date;
  control_state_cache_.assign(kCacheSize, 1, default_control_state_date);
}

void ControlHistoryStateManager::UpdateHistoryData(bool is_under_control,
                                                   double kappa_cmd,
                                                   double steer_pct_chassis,
                                                   double acc_target) {
  control_state_cache_.push_back({.is_under_control = is_under_control,
                                  .kappa_cmd = kappa_cmd,
                                  .steer_pct_chassis = steer_pct_chassis,
                                  .acc_target = acc_target});
}

}  // namespace control
}  // namespace qcraft
