#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_COMPARISON_TEST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_COMPARISON_TEST_UTIL_H_

#include <utility>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"

namespace qcraft {
namespace planner {

class CostComparisonTest {
 public:
  static constexpr int kSteps = 100;
  using Mfob = MixedFourthOrderBicycle<kSteps>;
  using Tob = ThirdOrderBicycle<kSteps>;

  using TobStateType = typename Tob::StateType;
  using TobControlType = typename Tob::ControlType;
  using TobStatesType = typename Tob::StatesType;
  using TobControlsType = typename Tob::ControlsType;
  using TobGType = typename Tob::GType;
  using TobDGDxType = typename Tob::DGDxType;
  using TobDGDuType = typename Tob::DGDuType;
  using TobDDGDxDxType = typename Tob::DDGDxDxType;
  using TobDDGDxDuType = typename Tob::DDGDxDuType;
  using TobDDGDuDxType = typename Tob::DDGDuDxType;
  using TobDDGDuDuType = typename Tob::DDGDuDuType;

  using MfobStateType = typename Mfob::StateType;
  using MfobControlType = typename Mfob::ControlType;
  using MfobStatesType = typename Mfob::StatesType;
  using MfobControlsType = typename Mfob::ControlsType;
  using MfobGType = typename Mfob::GType;
  using MfobDGDxType = typename Mfob::DGDxType;
  using MfobDGDuType = typename Mfob::DGDuType;
  using MfobDDGDxDxType = typename Mfob::DDGDxDxType;
  using MfobDDGDxDuType = typename Mfob::DDGDxDuType;
  using MfobDDGDuDxType = typename Mfob::DDGDuDxType;
  using MfobDDGDuDuType = typename Mfob::DDGDuDuType;

  static void CompareCostTest(Cost<Tob> *tob_cost, Cost<Mfob> *mfob_cost) {
    const std::vector<TobStateType> tob_states = {
        Tob::MakeState(1.0, 2.0, M_PI_4, 1.0, 0.5, 0.1, 1.0),
        Tob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        Tob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 5.0)};
    const std::vector<TobControlType> tob_controls = {
        Tob::MakeControl(0.0, 0.0), Tob::MakeControl(0.1, 0.0),
        Tob::MakeControl(0.0, 0.1), Tob::MakeControl(0.1, 0.1)};

    const std::vector<MfobStateType> mfob_states = {
        Mfob::MakeState(1.0, 2.0, M_PI_4, 1.0, 0.5, 0.1, 0.05, 1.0),
        Mfob::MakeState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0),
        Mfob::MakeState(0.3, 0.7, 0.9, 0.2, 0.1, 0.1, 0.05, 5.0)};
    const std::vector<MfobControlType> mfob_controls = {
        Mfob::MakeControl(0.0, 0.0), Mfob::MakeControl(0.1, 0.0),
        Mfob::MakeControl(0.0, 0.1), Mfob::MakeControl(0.1, 0.1)};

    for (int i = 0; i < tob_states.size(); ++i) {
      for (int j = 0; j < tob_controls.size(); ++j) {
        TobStatesType tob_xs;
        TobControlsType tob_us;
        MfobStatesType mfob_xs;
        MfobControlsType mfob_us;
        for (int k = 0; k < Tob::kHorizon; ++k) {
          Tob::SetStateAtStep(tob_states[i], k, &tob_xs);
          Tob::SetControlAtStep(tob_controls[j], k, &tob_us);
          Mfob::SetStateAtStep(mfob_states[i], k, &mfob_xs);
          Mfob::SetControlAtStep(mfob_controls[j], k, &mfob_us);
        }
        tob_cost->Update(tob_xs, tob_us, /*value_only=*/true);
        mfob_cost->Update(mfob_xs, mfob_us, /*value_only=*/true);

        constexpr double kEpsilon = 1e-9;
        for (int m = 0; m < Tob::kHorizon; ++m) {
          EXPECT_NEAR(tob_cost->EvaluateG(m, Tob::GetStateAtStep(tob_xs, m),
                                          Tob::GetControlAtStep(tob_us, m)),
                      mfob_cost->EvaluateG(m, Mfob::GetStateAtStep(mfob_xs, m),
                                           Mfob::GetControlAtStep(mfob_us, m)),
                      kEpsilon);
        }
      }
    }
  }
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_COST_COMPARISON_TEST_UTIL_H_
