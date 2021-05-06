#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_FORWARD_SPEED_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_FORWARD_SPEED_COST_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class ForwardSpeedCost : public Cost<PROB> {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  using DividedG = typename Cost<PROB>::DividedG;

  static constexpr double kNormalizedScale = 1000.0;
  explicit ForwardSpeedCost(
      std::string name = absl::StrCat(PROB::kProblemPrefix, "ForwardSpeedCost"),
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale) {}

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      res.AddSubG(/*idx=*/0, gain_[k] * Sqr(PROB::v(xs, k)));
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    return 0.5 * Cost<PROB>::scale() * gain_[k] * Sqr(PROB::StateGetV(x));
  }

  // Gradients with Superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    (*dgdx)[PROB::kStateVIndex] +=
        Cost<PROB>::scale() * gain_[k] * PROB::StateGetV(x);
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with Superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) +=
        Cost<PROB>::scale() * gain_[k];
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    for (int k = 0; k < PROB::kHorizon; ++k) {
      gain_[k] = PROB::v(xs, k) < 0.0 ? 1.0 : 0.0;
    }
  }

 private:
  // States.
  std::array<double, PROB::kHorizon> gain_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_FORWARD_SPEED_COST_H_
