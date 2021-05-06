#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_LINE_DEVIATION_COST_H_  // NOLINT
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_LINE_DEVIATION_COST_H_  // NOLINT

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class ReferenceLineDeviationCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 5.0;
  ReferenceLineDeviationCost(
      double path_gain, double end_state_gain,
      const std::vector<Vec2d> &ref_points,
      const CenterLineQueryHelper<PROB> *center_line_helper,
      std::vector<double> gains = {},
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "ReferenceLineDeviationCost"),
      double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        ref_gains_(std::move(gains)),
        path_gain_(path_gain),
        end_state_gain_(end_state_gain),
        center_line_helper_(center_line_helper) {
    if (center_line_helper_ == nullptr) {
      QCHECK_GT(ref_points.size(), 1);
      if (ref_gains_.empty()) ref_gains_.resize(ref_points.size() - 1, 1.0);
      QCHECK_EQ(ref_gains_.size(), ref_points.size() - 1);
      ref_path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(ref_points).value());
    }
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      res.AddSubG(/*idx=*/0, gains_[k] * Sqr(deviations_[k]));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    return 0.5 * Cost<PROB>::scale() * gains_[k] * Sqr(deviations_[k]);
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType &x,
                                  const ControlType &u,
                                  bool using_scale) const override {
    DividedG res(/*size=*/1);
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.AddSubG(/*idx=*/0, EvaluateG(k, x, u));
    if (!using_scale) {
      if (k != PROB::kHorizon - 1) {
        if (path_gain_ != 0.0) {
          res.Multi(1.0 / path_gain_);
        }
      }
    }
    return res;
  }
  // Gradients with superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    (*dgdx).template segment<2>(PROB::kStateXIndex) +=
        Cost<PROB>::scale() * gains_[k] * deviations_[k] *
        ref_normals_[k].transpose();
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex, PROB::kStateXIndex) +=
        Cost<PROB>::scale() * gains_[k] * ref_normals_[k] *
        ref_normals_[k].transpose();
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    if (center_line_helper_ == nullptr) {
      for (int k = 0; k < PROB::kHorizon; ++k) {
        const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
        FrenetCoordinate sl;
        Vec2d normal;
        std::pair<int, int> index_pair;
        double alpha = 0.0;
        ref_path_->XYToSL(pos, &sl, &normal, &index_pair, &alpha);
        ref_normals_[k] = normal;
        deviations_[k] = sl.l;
        gains_[k] = ref_gains_[index_pair.first] *
                    (k == PROB::kHorizon - 1 ? end_state_gain_ : path_gain_);
      }
    } else {
      const auto &rac_index_pairs = center_line_helper_->index_pairs();
      const auto &rac_s_l_list = center_line_helper_->s_l_list();
      const auto &rac_normals = center_line_helper_->normals();
      for (int k = 0; k < PROB::kHorizon; ++k) {
        deviations_[k] = rac_s_l_list[k].l;
        ref_normals_[k] = rac_normals[k];
        gains_[k] = ref_gains_[rac_index_pairs[k].first] *
                    (k == PROB::kHorizon - 1 ? end_state_gain_ : path_gain_);
      }
    }
  }

 private:
  std::vector<double> ref_gains_;
  double path_gain_ = 0.0;
  double end_state_gain_ = 0.0;
  std::unique_ptr<FrenetFrame> ref_path_;
  const CenterLineQueryHelper<PROB> *center_line_helper_;

  // States.
  std::array<Vec2d, PROB::kHorizon> ref_normals_;
  std::array<double, PROB::kHorizon> deviations_;
  std::array<double, PROB::kHorizon> gains_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_REFERENCE_LINE_DEVIATION_COST_H_
