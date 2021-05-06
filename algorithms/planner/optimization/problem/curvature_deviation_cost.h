#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CURVATURE_DEVIATION_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CURVATURE_DEVIATION_COST_H_

#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/math/util.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class CurvatureDeviationCost : public Cost<PROB> {
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

  CurvatureDeviationCost(std::vector<double> ref_path_s,
                         std::vector<double> ref_path_kappa,
                         const double weight, const double weight_decay_rate,
                         std::string name = absl::StrCat(
                             PROB::kProblemPrefix, "CurvatureDeviationCost"),
                         double scale = 1.0)
      : Cost<PROB>(std::move(name), scale),
        ref_path_s_(std::move(ref_path_s)),
        ref_path_kappa_(std::move(ref_path_kappa)),
        weight_(weight),
        weight_decay_rate_(weight_decay_rate) {
    QCHECK_EQ(ref_path_s_.size(), ref_path_kappa_.size());
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      res.AddSubG(/*idx=*/0, weights_[k] * Sqr(kappa_diff_[k]));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    return 0.5 * Cost<PROB>::scale() * weights_[k] * Sqr(kappa_diff_[k]);
  }

  // Gradients with Superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    (*dgdx)[PROB::kStateKappaIndex] +=
        Cost<PROB>::scale() * weights_[k] * kappa_diff_[k];
    (*dgdx)[PROB::kStateSIndex] +=
        (weights_[k] > 0.0
             ? 0.5 * Cost<PROB>::scale() *
                   (weight_ * weight_decay_rate_ * Sqr(kappa_diff_[k]) -
                    2.0 * weights_[k] * kappa_diff_[k] * factor_[k])
             : 0.0);
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with Superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    const double weight_k = weights_[k];
    const double factor_k = factor_[k];
    const double weight_mul_factor = weight_k * factor_k;
    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateKappaIndex) +=
        Cost<PROB>::scale() * weight_k;
    (*ddgdxdx)(PROB::kStateKappaIndex, PROB::kStateSIndex) +=
        (weight_k > 0.0 ? Cost<PROB>::scale() *
                              (weight_ * weight_decay_rate_ * kappa_diff_[k] -
                               weight_mul_factor)
                        : 0.0);
    (*ddgdxdx)(PROB::kStateSIndex, PROB::kStateKappaIndex) +=
        (weight_k > 0.0 ? Cost<PROB>::scale() *
                              (weight_ * weight_decay_rate_ * kappa_diff_[k] -
                               weight_mul_factor)
                        : 0.0);
    (*ddgdxdx)(PROB::kStateSIndex, PROB::kStateSIndex) +=
        (weight_k > 0.0 ? 0.5 * Cost<PROB>::scale() *
                              (4.0 * weight_ * weight_decay_rate_ *
                                   kappa_diff_[k] * (-factor_k) +
                               2.0 * weight_k * factor_k * factor_k)
                        : 0.0);
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    for (int i = 0; i < PROB::kHorizon; ++i) {
      // Binary search corresponding ref_path point by s, remember to
      // synchronize s between two frames before pass in ref_path_s.
      int index =
          std::distance(ref_path_s_.begin(),
                        std::upper_bound(ref_path_s_.begin(), ref_path_s_.end(),
                                         PROB::s(xs, i)));
      index = std::max(0, index - 1);
      if (index == ref_path_s_.size() - 1) {
        kappa_diff_[i] = 0.0;
        weights_[i] = 0.0;
        factor_[i] = 0.0;
      } else {
        double s_base = ref_path_s_[index + 1] - ref_path_s_[index];
        double s_diff = PROB::s(xs, i) - ref_path_s_[index];
        double alpha = s_diff / s_base;
        if (std::abs(s_base) < 1e-9) {
          alpha = 0.0;
          factor_[i] = 0.0;
        } else {
          factor_[i] =
              (ref_path_kappa_[index + 1] - ref_path_kappa_[index]) / s_base;
        }
        kappa_diff_[i] =
            PROB::kappa(xs, i) -
            (alpha * (ref_path_kappa_[index + 1] - ref_path_kappa_[index]) +
             ref_path_kappa_[index]);

        weights_[i] =
            weight_ * std::max(0.0, weight_decay_rate_ * PROB::s(xs, i) + 1.0);
      }
    }
  }

 private:
  std::vector<double> ref_path_s_;
  std::vector<double> ref_path_kappa_;
  double weight_;
  double weight_decay_rate_;

  // States.
  std::array<double, PROB::kHorizon> weights_;
  std::array<double, PROB::kHorizon> kappa_diff_;
  std::array<double, PROB::kHorizon> factor_;
};
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CURVATURE_DEVIATION_COST_H_
