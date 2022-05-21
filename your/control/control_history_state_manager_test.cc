#include "onboard/control/control_history_state_manager.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace control {
namespace {

constexpr double kEpsilon = 1e-6;

TEST(ControlHistoryStateManagerTest, DefaultCacheTest) {
  ControlHistoryStateManager Control_history_state_mgr;
  EXPECT_TRUE(!Control_history_state_mgr.GetControlStateCache().empty());
  EXPECT_NEAR(Control_history_state_mgr.GetControlStateCache().back().kappa_cmd,
              0.0, kEpsilon);
}

TEST(ControlHistoryStateManagerTest, UpdateCacheTest) {
  ControlHistoryStateManager Control_history_state_mgr;
  constexpr double kGradient = 0.01;
  for (int i = 0; i < 300; ++i) {
    Control_history_state_mgr.UpdateHistoryData(true, i * kGradient, 0, 0);

    const int size = Control_history_state_mgr.GetControlStateCache().size();
    const double front_cache_expected =
        std::max((i + 1 - size) * kGradient, 0.0);
    const double back_cache_expected = i * kGradient;

    EXPECT_NEAR(
        Control_history_state_mgr.GetControlStateCache().back().kappa_cmd,
        back_cache_expected, kEpsilon);
    EXPECT_NEAR(
        Control_history_state_mgr.GetControlStateCache().front().kappa_cmd,
        front_cache_expected, kEpsilon);
  }
}

}  // namespace
}  // namespace control
}  // namespace qcraft
