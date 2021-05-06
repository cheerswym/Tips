#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_HOOK_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_HOOK_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "onboard/math/eigen.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class DdpOptimizerHook {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  struct OptimizerInspector {
    double cost;  // Updated before OnLineSearchIterationEnd().
    double js0;   // Updated in OnIterationEnd().

    absl::flat_hash_map<std::string, double>
        cost_map;  // Updated at iterations.
  };

  virtual ~DdpOptimizerHook() {}
  virtual void OnSolveStart(const StatesType &xs, const ControlsType &us,
                            const OptimizerInspector &oi) {}
  virtual void OnSolveEnd(const StatesType &xs, const ControlsType &us,
                          const OptimizerInspector &oi) {}
  virtual void OnIterationStart(int iter, const StatesType &xs,
                                const ControlsType &us,
                                const OptimizerInspector &oi) {}
  virtual void OnIterationEnd(int iter, const StatesType &xs,
                              const ControlsType &us,
                              const OptimizerInspector &oi) {}
  virtual void OnLineSearchIterationStart(int iter, const StatesType &xs,
                                          const ControlsType &us,
                                          const StatesType &full_dxs,
                                          const ControlsType &full_dus,
                                          double alpha,
                                          const OptimizerInspector &oi) {}
  virtual void OnLineSearchIterationEnd(int iter, const StatesType &xs,
                                        const ControlsType &us,
                                        const StatesType &full_dxs,
                                        const ControlsType &full_dus,
                                        double alpha, double cost,
                                        const OptimizerInspector &oi) {}
  virtual void OnStepSizeAdjustmentIterationStart(
      int iter, const StatesType &xs, const ControlsType &us, int k_stepsize,
      const OptimizerInspector &oi) {}
  virtual void OnStepSizeAdjustmentIterationEnd(int iter, const StatesType &xs,
                                                const ControlsType &us,
                                                int k_stepsize, double cost,
                                                const OptimizerInspector &oi) {}
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_HOOK_H_
