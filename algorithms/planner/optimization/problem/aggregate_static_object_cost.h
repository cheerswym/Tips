#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AGGREGATE_STATIC_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AGGREGATE_STATIC_OBSTACLE_COST_H_

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/util/min_segment_distance_problem.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class AggregateStaticObjectCost : public Cost<PROB> {
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

  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  static constexpr double kCutoffDistance = 50.0;
  static constexpr double kNormalizedScale = 1.0;
  AggregateStaticObjectCost(
      const std::vector<Segment2d> &segments, double l,
      std::vector<double> buffers, std::vector<double> gains,
      std::vector<std::string> sub_names, int num_objects,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "AggregateStaticObjectCost"),
      double scale = 1.0, bool enable_fast_math = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, enable_fast_math),
        static_object_boundary_(CreateNamedSegments(segments),
                                /*use_qtfm=*/false,
                                /*cutoff_distance=*/kCutoffDistance),
        num_objects_(num_objects),
        l_(l),
        buffers_(std::move(buffers)),
        gains_(std::move(gains)),
        sub_names_(std::move(sub_names)) {
    QCHECK_EQ(buffers_.size(), gains_.size());
    QCHECK_EQ(buffers_.size(), sub_names_.size());

    penetrations_.resize(num_objects_);
    penetration_jacobians_.resize(num_objects_,
                                  PenetrationJacobianType::Zero());
    penetration_hessians_.resize(num_objects_, PenetrationHessianType::Zero());
    for (int i = 0, n = num_objects_; i < n; ++i) {
      // Object prediction time may be shorter than ddp time horizon.
      auto &penetrations_i = penetrations_[i];
      penetrations_i.reserve(buffers_.size());
      for (int k = 0; k < buffers_.size(); ++k) {
        penetrations_i.push_back(std::numeric_limits<double>::infinity());
      }
    }
  }

  const std::vector<std::vector<double>> &penetrations() const {
    return penetrations_;
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (k >= num_objects_) break;
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(i, gains_[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType &x,
                                  const ControlType &u,
                                  bool using_scale) const override {
    DCHECK_GE(k, 0);
    std::vector<double> gains(gains_.size());
    if (using_scale) {
      gains = gains_;
    } else {
      std::fill(gains.begin(), gains.end(), 1.0);
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    if (k >= num_objects_) return res;
    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        res.AddSubG(
            i, 0.5 * Cost<PROB>::scale() * gains[i] * Sqr(penetrations_k[i]));
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return 0.0;
    double g = 0.0;
    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        g += 0.5 * Cost<PROB>::scale() * gains_[i] * Sqr(penetrations_k[i]);
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;

    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto &penetration_jacobians_k = penetration_jacobians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *dgdx += Cost<PROB>::scale() * gains_[i] * penetrations_k[i] *
                 penetration_jacobians_k;
      }
    }
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;

    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto &penetration_jacobians_k = penetration_jacobians_[k];
    const auto &penetration_hessians_k = penetration_hessians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *ddgdxdx +=
            Cost<PROB>::scale() * gains_[i] *
            (penetrations_k[i] * penetration_hessians_k +
             penetration_jacobians_k.transpose() * penetration_jacobians_k);
      }
    }
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType & /*us*/,
              bool value_only) override {
    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (k >= num_objects_) break;
      auto penetrations_k = absl::MakeSpan(penetrations_[k]);
      auto &penetration_jacobians_k = penetration_jacobians_[k];
      auto &penetration_hessians_k = penetration_hessians_[k];

      const auto x0 = PROB::GetStateAtStep(xs, k);
      const Vec2d av_tangent =
          Cost<PROB>::enable_fast_math()
              ? Vec2d::FastUnitFromAngle(PROB::StateGetTheta(x0))
              : Vec2d::UnitFromAngle(PROB::StateGetTheta(x0));
      VLOG(3) << "Step " << k;
      VLOG(4) << "x0 = " << x0.transpose()
              << " av_tangent = " << av_tangent.transpose();
      // Vehicle position, rear center.
      const Vec2d pos = PROB::pos(xs, k);

      if (value_only) {
        const double msd = CalculateOneCircleMsd(
            pos.x(), pos.y(), av_tangent.y(), av_tangent.x(), l_, 0.0);
        for (int i = 0; i < penetrations_k.size(); ++i) {
          penetrations_k[i] = msd - buffers_[i];
        }
      } else {
        penetration_jacobians_k.setZero();
        penetration_hessians_k.setZero();
        double *df_dx = &penetration_jacobians_k(PROB::kStateXIndex);
        double *df_dy = &penetration_jacobians_k(PROB::kStateYIndex);
        double *df_dtheta = &penetration_jacobians_k(PROB::kStateThetaIndex);

        double *d2f_dx2 =
            &penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex);
        double *d2f_dy2 =
            &penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex);
        double *d2f_dtheta2 = &penetration_hessians_k(PROB::kStateThetaIndex,
                                                      PROB::kStateThetaIndex);
        double *d2f_dx_dy =
            &penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
        double *d2f_dx_dtheta =
            &penetration_hessians_k(PROB::kStateXIndex, PROB::kStateThetaIndex);
        double *d2f_dy_dtheta =
            &penetration_hessians_k(PROB::kStateYIndex, PROB::kStateThetaIndex);
        double msd = 0.0;
        CalculateOneCircleMsdWithDerivative(
            pos.x(), pos.y(), av_tangent.y(), av_tangent.x(), l_, 0.0, &msd,
            df_dx, df_dy, df_dtheta, d2f_dx2, d2f_dy2, d2f_dtheta2, d2f_dx_dy,
            d2f_dx_dtheta, d2f_dy_dtheta);
        penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
        penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateXIndex) =
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateThetaIndex);
        penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateYIndex) =
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateThetaIndex);

        for (int i = 0; i < penetrations_k.size(); ++i) {
          penetrations_k[i] = msd - buffers_[i];
        }
      }
    }
  }

 private:
  // TODO(huaiyuan): Use separate vector for segment and string in MSD,
  // so that user can avoid fabrication.
  static std::vector<std::pair<std::string, Segment2d>> CreateNamedSegments(
      const std::vector<Segment2d> &segments) {
    std::vector<std::pair<std::string, Segment2d>> named_segments;
    named_segments.reserve(segments.size());
    for (int i = 0; i < segments.size(); ++i) {
      named_segments.emplace_back(absl::StrFormat("%d", i), segments[i]);
    }
    return named_segments;
  }

  void CalculateOneCircleMsdWithDerivative(
      double x, double y, double sin_theta, double cos_theta,
      double forward_shift, double rightward_shift, double *msd, double *df_dx,
      double *df_dy, double *df_dtheta, double *d2f_dx2, double *d2f_dy2,
      double *d2f_dtheta2, double *d2f_dx_dy, double *d2f_dx_dtheta,
      double *d2f_dy_dtheta) const {
    DCHECK_NOTNULL(msd);
    DCHECK_NOTNULL(df_dx);
    DCHECK_NOTNULL(df_dy);
    DCHECK_NOTNULL(df_dtheta);
    DCHECK_NOTNULL(d2f_dx2);
    DCHECK_NOTNULL(d2f_dy2);
    DCHECK_NOTNULL(d2f_dtheta2);
    DCHECK_NOTNULL(d2f_dx_dy);
    DCHECK_NOTNULL(d2f_dx_dtheta);
    DCHECK_NOTNULL(d2f_dy_dtheta);

    MinSegmentDistanceProblem::SecondOrderDerivativeType der;
    const double xr =
        x + forward_shift * cos_theta + rightward_shift * sin_theta;
    const double yr =
        y + forward_shift * sin_theta - rightward_shift * cos_theta;
    const double min_dis =
        static_object_boundary_.EvaluateWithSecondOrderDerivatives(
            Vec2d{xr, yr}, &der);

    *msd = min_dis;
    if (*msd > 0) {
      const double delta_y = y - yr;
      const double delta_x = x - xr;
      *df_dx = der.df_dx;
      *df_dy = der.df_dy;
      *df_dtheta = delta_y * der.df_dx - delta_x * der.df_dy;
      *d2f_dx2 = der.d2f_dx_dx;
      *d2f_dx_dy = der.d2f_dx_dy;
      *d2f_dx_dtheta = delta_y * der.d2f_dx_dx - delta_x * der.d2f_dx_dy;
      *d2f_dy2 = der.d2f_dy_dy;
      *d2f_dy_dtheta = delta_y * der.d2f_dx_dy - delta_x * der.d2f_dy_dy;
      *d2f_dtheta2 =
          delta_y * (delta_y * der.d2f_dx_dx - delta_x * der.d2f_dx_dy) +
          delta_y * der.df_dy + delta_x * der.df_dx -
          delta_x * (delta_y * der.d2f_dx_dy - delta_x * der.d2f_dy_dy);
    } else {
      // Using following equation would be faster when no penatration.
      *df_dx = 0.0;
      *df_dy = 0.0;
      *df_dtheta = 0.0;
      *d2f_dx2 = 0.0;
      *d2f_dx_dy = 0.0;
      *d2f_dx_dtheta = 0.0;
      *d2f_dy2 = 0.0;
      *d2f_dy_dtheta = 0.0;
      *d2f_dtheta2 = 0.0;
    }
  }

  double CalculateOneCircleMsd(double x, double y, double sin_theta,
                               double cos_theta, double forward_shift,
                               double rightward_shift) const {
    const double xr =
        x + forward_shift * cos_theta + rightward_shift * sin_theta;
    const double yr =
        y + forward_shift * sin_theta - rightward_shift * cos_theta;
    const double min_dis = static_object_boundary_.Evaluate(Vec2d{xr, yr});
    return min_dis;
  }

  MinSegmentDistanceProblem static_object_boundary_;
  int num_objects_;

  // Distances from control points to RAC on vehicle longitudinal axis.
  double l_;

  // Buffer to static object
  std::vector<double> buffers_;
  std::vector<double> gains_;
  std::vector<std::string> sub_names_;

  // States.
  std::vector<std::vector<double>> penetrations_;
  std::vector<PenetrationJacobianType> penetration_jacobians_;
  std::vector<PenetrationHessianType> penetration_hessians_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_OBSTACLE_COST_H_
