#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_COST_MANAGER_HOOK_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_COST_MANAGER_HOOK_H_

#include <vector>

#include "onboard/math/eigen.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_hook.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/cost_helper.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class DdpCostManagerHook : public DdpOptimizerHook<PROB> {
 public:
  using OptimizerInspector =
      typename DdpOptimizerHook<PROB>::OptimizerInspector;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  void OnSolveStart(const StatesType &xs, const ControlsType &us,
                    const OptimizerInspector &oi) override {
    for (auto *helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto *cost : costs_) {
      cost->Update(xs, us, /*value_only=*/false);
    }
  }

  void OnIterationStart(int iter, const StatesType &xs, const ControlsType &us,
                        const OptimizerInspector &oi) override {
    for (auto *helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto *cost : costs_) {
      cost->Update(xs, us, /*value_only=*/false);
    }
  }

  void OnLineSearchIterationStart(int iter, const StatesType &xs,
                                  const ControlsType &us,
                                  const StatesType &full_dxs,
                                  const ControlsType &full_dus, double alpha,
                                  const OptimizerInspector &oi) override {
    for (auto *helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto *cost : costs_) {
      cost->Update(xs, us, /*value_only=*/true);
    }
  }

  void OnStepSizeAdjustmentIterationStart(
      int iter, const StatesType &xs, const ControlsType &us, int k_stepsize,
      const OptimizerInspector &oi) override {
    for (auto *helper : helpers_) {
      helper->Update(xs, us);
    }
    for (auto *cost : costs_) {
      cost->Update(xs, us, /*value_only=*/true);
    }
  }

  void AddCost(Cost<PROB> *cost) {
    QCHECK(cost != nullptr);
    costs_.push_back(cost);
  }
  void AddCostHelper(CostHelper<PROB> *helper) {
    QCHECK(helper != nullptr);
    helpers_.push_back(helper);
  }

 private:
  std::vector<Cost<PROB> *> costs_;
  std::vector<CostHelper<PROB> *> helpers_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_COST_MANAGER_HOOK_H_
