#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_END_HEADING_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_END_HEADING_COST_H_

#include <algorithm>
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
class EndHeadingCost : public Cost<PROB> {
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
  EndHeadingCost(const std::vector<Vec2d> &ref_points,
                 const CenterLineQueryHelper<PROB> *center_line_helper,
                 std::string name = absl::StrCat(PROB::kProblemPrefix,
                                                 "EndHeadingCost"),
                 double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        center_line_helper_(center_line_helper) {
    if (center_line_helper_ == nullptr) {
      QCHECK_GT(ref_points.size(), 1);
      ref_path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(ref_points).value());
      // Size may be smaller than ref_points due to Frenet frame deduplication.
      const auto &ref_tangents = ref_path_->tangents();
      ref_thetas_.reserve(ref_tangents.size() - 1);
      for (int i = 0; i + 1 < ref_tangents.size(); ++i) {
        ref_thetas_.push_back(ref_tangents[i].FastAngle());
      }
    } else {
      const auto &ref_tangents = center_line_helper_->tangents();
      ref_thetas_.reserve(ref_tangents.size() - 1);
      for (int i = 0; i + 1 < ref_tangents.size(); ++i) {
        ref_thetas_.push_back(ref_tangents[i].FastAngle());
      }
    }
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k + 1 < PROB::kHorizon; ++k) {
      res.AddSubG(/*idx=*/0, Sqr(deviations_[k]));
    }
    res.Multi(kPathGain * 0.5 * Cost<PROB>::scale());
    res.AddSubG(/*idx=*/0, 0.5 * kEndStateGain * Cost<PROB>::scale() *
                               Sqr(deviations_[PROB::kHorizon - 1]));
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    const double gain = k < PROB::kHorizon - 1 ? kPathGain : kEndStateGain;
    return 0.5 * gain * Cost<PROB>::scale() * Sqr(deviations_[k]);
  }

  // Gradients with Superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    const double gain = (k < PROB::kHorizon - 1 ? kPathGain : kEndStateGain) *
                        Cost<PROB>::scale();
    (*dgdx)(PROB::kStateThetaIndex) += gain * deviations_[k];
    (*dgdx).template segment<2>(PROB::kStateXIndex) +=
        gain * deviations_[k] * factors_[k];
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with Superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    const double gain = (k < PROB::kHorizon - 1 ? kPathGain : kEndStateGain) *
                        Cost<PROB>::scale();
    (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) += gain;
    const auto &factor_k = factors_[k];
    const Vec2d ddgdthetadxy = gain * factor_k;
    const Vec2d ddgdxydxy = gain * Vec2d(Sqr(factor_k.x()), Sqr(factor_k.y()));
    const double ddgdxdy = gain * factor_k.x() * factor_k.y();
    (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateXIndex) += ddgdthetadxy[0];
    (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateYIndex) += ddgdthetadxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateThetaIndex) += ddgdthetadxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateThetaIndex) += ddgdthetadxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateXIndex) += ddgdxydxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateYIndex) += ddgdxydxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateYIndex) += ddgdxdy;
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateXIndex) += ddgdxdy;
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    constexpr double kEps = 1e-7;
    if (center_line_helper_ == nullptr) {
      const std::vector<double> &s_knot = ref_path_->s_knots();
      for (int k = 0; k < PROB::kHorizon; ++k) {
        const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
        FrenetCoordinate sl;
        Vec2d normal;
        int index = 0;
        double alpha = 0.0;
        ref_path_->XYToSL(pos, &sl, &normal, &index, &alpha);
        QCHECK_LT(index, ref_thetas_.size());
        int next_index =
            std::min(index + 1, static_cast<int>(ref_thetas_.size() - 1));
        const auto curent_ref_theta = ref_thetas_[index];
        const auto next_ref_theta = ref_thetas_[next_index];
        const double ref_theta =
            LerpAngle(curent_ref_theta, next_ref_theta, alpha);
        const double arc_length = s_knot[next_index] - s_knot[index];
        if (arc_length < kEps) {
          factors_[k] = Vec2d(0.0, 0.0);
        } else {
          factors_[k] = NormalizeAngle(next_ref_theta - curent_ref_theta) *
                        normal.Perp() / arc_length;
        }
        deviations_[k] = NormalizeAngle(
            PROB::StateGetTheta(PROB::GetStateAtStep(xs, k)) - ref_theta);
      }
    } else {
      const auto &rac_index_pairs = center_line_helper_->index_pairs();
      const auto &alphas = center_line_helper_->alphas();
      const std::vector<double> &s_knot = center_line_helper_->s_knots();
      const auto &normals = center_line_helper_->normals();
      for (int k = 0; k < PROB::kHorizon; ++k) {
        const int index = rac_index_pairs[k].first;
        int next_index =
            std::min(index + 1, static_cast<int>(ref_thetas_.size() - 1));
        const auto curent_ref_theta = ref_thetas_[index];
        const auto next_ref_theta = ref_thetas_[next_index];
        const double ref_theta =
            LerpAngle(curent_ref_theta, next_ref_theta, alphas[k]);
        const double arc_length = s_knot[next_index] - s_knot[index];
        if (arc_length < kEps) {
          factors_[k] = Vec2d(0.0, 0.0);
        } else {
          factors_[k] = NormalizeAngle(next_ref_theta - curent_ref_theta) *
                        normals[k].Perp() / arc_length;
        }
        deviations_[k] = NormalizeAngle(
            PROB::StateGetTheta(PROB::GetStateAtStep(xs, k)) - ref_theta);
      }
    }
  }

 protected:
  static constexpr double kPathGain = 2.0;
  static constexpr double kEndStateGain = 10.0;

 private:
  std::vector<double> ref_thetas_;
  std::unique_ptr<FrenetFrame> ref_path_;
  const CenterLineQueryHelper<PROB> *center_line_helper_;

  // States.
  std::array<double, PROB::kHorizon> deviations_;
  std::array<Vec2d, PROB::kHorizon> factors_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_END_HEADING_COST_H_
