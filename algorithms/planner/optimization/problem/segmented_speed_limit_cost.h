#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {

// TODO(Fang) use hysteresis in gain switches.
template <typename PROB>
class SegmentedSpeedLimitCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 0.01;
  explicit SegmentedSpeedLimitCost(
      const std::vector<Vec2d> &ref_points, std::vector<double> speed_limits,
      const std::vector<Vec2d> &free_ref_points,
      std::vector<double> free_speed_limits, int free_index,
      const PiecewiseLinearFunction<double> &under_speed_gain_compensation_plf,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "SegmentedSpeedLimitCost"),
      double scale = 1.0, double stop_speed_gain = kDefaultStopSpeedCostGain,
      double over_speed_gain = kDefaultOverSpeedCostGain,
      double under_speed_gain = kDefaultUnderSpeedCostGain,
      bool use_qtfm = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        speed_limits_(std::move(speed_limits)),
        free_speed_limits_(std::move(free_speed_limits)),
        free_index_(free_index),
        under_speed_gain_compensation_plf_(under_speed_gain_compensation_plf),
        stop_speed_gain_(stop_speed_gain),
        over_speed_gain_(over_speed_gain),
        under_speed_gain_(under_speed_gain) {
    QCHECK_GT(ref_points.size(), 1);
    QCHECK_EQ(ref_points.size(), speed_limits_.size() + 1);
    QCHECK_GT(free_ref_points.size(), 1);
    QCHECK_EQ(free_ref_points.size(), free_speed_limits_.size() + 1);
    if (use_qtfm) {
      ref_path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
          BuildQtfmEnhancedKdTreeFrenetFrame(ref_points).value());
    } else {
      ref_path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(ref_points).value());
    }
    // QTFM frenet frame is not covering speed limits beyond the drive passage
    // as there shouldn't be many querry outside drive passage.
    free_ref_path_ = std::make_unique<KdTreeFrenetFrame>(
        BuildKdTreeFrenetFrame(free_ref_points).value());
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      res.AddSubG(/*idx=*/0, gain_[k] * Sqr(speed_diff_[k]));
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    return 0.5 * Cost<PROB>::scale() * gain_[k] * Sqr(speed_diff_[k]);
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    const double speed_diff_k = speed_diff_[k];
    const auto &speed_limit_factor_k = speed_limit_factor_[k];
    const double dgdv = Cost<PROB>::scale() * gain_[k] * speed_diff_k;
    (*dgdx)[PROB::kStateVIndex] += dgdv;
    (*dgdx).template segment<2>(PROB::kStateXIndex) +=
        (dgdv +
         0.5 * Cost<PROB>::scale() * Sqr(speed_diff_k) * gain_factor_[k]) *
        speed_limit_factor_k;
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    const double gain = Cost<PROB>::scale() * gain_[k];
    const auto &speed_limit_factor_k = speed_limit_factor_[k];
    const double speed_diff_k = speed_diff_[k];
    const double gain_factor_k = gain_factor_[k];
    const double scale_multi_factor = Cost<PROB>::scale() * gain_factor_k;
    const double scale_multi_factor_muiti_speed_diff =
        scale_multi_factor * speed_diff_k;
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateVIndex) += gain;
    const Vec2d ddgdvdxy =
        (gain + scale_multi_factor_muiti_speed_diff) * speed_limit_factor_k;
    const Vec2d factor_multi =
        Vec2d(Sqr(speed_limit_factor_k.x()), Sqr(speed_limit_factor_k.y()));
    const Vec2d ddgdxydxy =
        (gain + 2.0 * scale_multi_factor_muiti_speed_diff) * factor_multi;
    const double ddgdxdy = (gain + 2.0 * scale_multi_factor_muiti_speed_diff) *
                           speed_limit_factor_k.x() * speed_limit_factor_k.y();
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateXIndex) = ddgdvdxy[0];
    (*ddgdxdx)(PROB::kStateVIndex, PROB::kStateYIndex) = ddgdvdxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateVIndex) = ddgdvdxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateVIndex) = ddgdvdxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateXIndex) = ddgdxydxy[0];
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateYIndex) = ddgdxydxy[1];
    (*ddgdxdx)(PROB::kStateXIndex, PROB::kStateYIndex) = ddgdxdy;
    (*ddgdxdx)(PROB::kStateYIndex, PROB::kStateXIndex) = ddgdxdy;
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    // Compensation for low forward force when speed limit is low, by increasing
    // the under-speed gain.
    constexpr double kEps = 1e-7;
    const std::vector<double> &s_knot = ref_path_->s_knots();
    for (int k = 0; k < PROB::kHorizon && k <= free_index_; ++k) {
      const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;
      ref_path_->XYToSL(pos, &sl, &normal, &index_pair, &alpha);
      const int next_index = std::min(
          index_pair.second, static_cast<int>(speed_limits_.size() - 1));
      const double speed_limit = Lerp(speed_limits_[index_pair.first],
                                      speed_limits_[next_index], alpha);
      const double speed_diff = PROB::v(xs, k) - speed_limit;
      speed_diff_[k] = PROB::v(xs, k) - speed_limit;
      const double length = s_knot[next_index] - s_knot[index_pair.first];
      if (length < kEps) {
        speed_limit_factor_[k] = Vec2d(0.0, 0.0);
      } else {
        speed_limit_factor_[k] =
            (speed_limits_[next_index] - speed_limits_[index_pair.first]) *
            normal.Perp() / length;
      }
      const double under_speed_gain_with_compensation =
          under_speed_gain_ * under_speed_gain_compensation_plf_(speed_limit);
      if (speed_limit == 0.0) {
        gain_[k] = stop_speed_gain_;
        gain_factor_[k] = 0.0;
      } else {
        gain_[k] = speed_diff > 0.0 ? over_speed_gain_
                                    : under_speed_gain_with_compensation;
        gain_factor_[k] =
            speed_diff > 0.0
                ? 0.0
                : -under_speed_gain_ *
                      under_speed_gain_compensation_plf_.EvaluateSlope(
                          speed_limit);
      }
    }
    const std::vector<double> &free_s_knot = free_ref_path_->s_knots();
    for (int k = free_index_ + 1; k < PROB::kHorizon; ++k) {
      const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;
      free_ref_path_->XYToSL(pos, &sl, &normal, &index_pair, &alpha);
      const int next_index = std::min(
          index_pair.second, static_cast<int>(free_speed_limits_.size() - 1));
      const double speed_limit = Lerp(free_speed_limits_[index_pair.first],
                                      free_speed_limits_[next_index], alpha);
      const double speed_diff = PROB::v(xs, k) - speed_limit;
      speed_diff_[k] = PROB::v(xs, k) - speed_limit;
      const double length =
          free_s_knot[next_index] - free_s_knot[index_pair.first];
      if (length < kEps) {
        speed_limit_factor_[k] = Vec2d(0.0, 0.0);
      } else {
        speed_limit_factor_[k] = (free_speed_limits_[next_index] -
                                  free_speed_limits_[index_pair.first]) *
                                 normal.Perp() / length;
      }
      const double under_speed_gain_with_compensation =
          under_speed_gain_ * under_speed_gain_compensation_plf_(speed_limit);
      if (speed_limit == 0.0) {
        gain_[k] = stop_speed_gain_;
        gain_factor_[k] = 0.0;
      } else {
        gain_[k] = speed_diff > 0.0 ? over_speed_gain_
                                    : under_speed_gain_with_compensation;
        gain_factor_[k] =
            speed_diff > 0.0
                ? 0.0
                : -under_speed_gain_ *
                      under_speed_gain_compensation_plf_.EvaluateSlope(
                          speed_limit);
      }
    }
  }

 protected:
  static constexpr double kDefaultStopSpeedCostGain = 800.0;
  static constexpr double kDefaultOverSpeedCostGain = 100.0;
  static constexpr double kDefaultUnderSpeedCostGain = 1.0;

 private:
  std::vector<double> speed_limits_;
  std::vector<double> free_speed_limits_;
  int free_index_;  // State index beyond this value uses free speed limit.
  PiecewiseLinearFunction<double> under_speed_gain_compensation_plf_;
  double stop_speed_gain_;
  double over_speed_gain_;
  double under_speed_gain_;
  std::unique_ptr<FrenetFrame> ref_path_;
  std::unique_ptr<FrenetFrame> free_ref_path_;

  // States.
  std::array<double, PROB::kHorizon> speed_diff_;
  std::array<double, PROB::kHorizon> gain_;
  std::array<Vec2d, PROB::kHorizon> speed_limit_factor_;
  std::array<double, PROB::kHorizon> gain_factor_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_SEGMENTED_SPEED_LIMIT_COST_H_
