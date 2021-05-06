#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_STATIC_BOUNDARY_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_STATIC_BOUNDARY_COST_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/util/min_segment_distance_problem.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// Use 3 circles to represent ADV
// static boundary are storaged in a min_segment_distance_problem
// Will avoid ADV from hitting static boundary
template <typename PROB>
class MsdStaticBoundaryCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 10.0;
  MsdStaticBoundaryCost(
      VehicleGeometryParamsProto vehicle_geometry_params,
      MinSegmentDistanceProblem curb_msd, std::vector<std::string> sub_names,
      const std::vector<double>& cascade_buffers,
      const std::vector<double>& cascade_gains,
      const std::vector<Vec2d>& circle_center_offsets,
      const std::vector<double>& circle_radiuses, int effect_index,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "MsdStaticBoundaryCost"),
      double scale = 1.0, bool enable_fast_math = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, enable_fast_math),
        vehicle_geometry_params_(std::move(vehicle_geometry_params)),
        curb_msd_(std::move(curb_msd)),
        cascade_buffers_(std::move(cascade_buffers)),
        cascade_gains_(std::move(cascade_gains)),
        circle_center_offsets_(std::move(circle_center_offsets)),
        circle_radiuses_(std::move(circle_radiuses)),
        sub_names_(std::move(sub_names)),
        enable_fast_math_(enable_fast_math),
        effect_index_(effect_index) {
    const int n = cascade_buffers.size();
    QCHECK_GT(n, 0);
    QCHECK_EQ(n, cascade_gains_.size());
    QCHECK_EQ(n, sub_names_.size());
    QCHECK_LE(effect_index_, PROB::kHorizon);

    cascade_gains_final_.reserve(n);

    double max_radius = 0.0;
    for (int i = 0; i < n; ++i) {
      cascade_gains_final_.push_back(cascade_gains_[i] * Cost<PROB>::scale() *
                                     0.5);
      max_radius = std::max(circle_radiuses_.back(), max_radius);
    }

    QCHECK_LE(max_radius, curb_msd_.cutoff_distance());
    QCHECK_EQ(circle_center_offsets_.size(), circle_radiuses_.size());
  }

  DividedG SumGForAllSteps(const StatesType& xs,
                           const ControlsType& us) const override {
    DividedG res(sub_names_.size());
    for (int k = 0; k < effect_index_; ++k) {
      const auto& res_k = updated_results_[k].divided_gs;
      for (int i = 0; i < res_k.size(); ++i) {
        res.AddSubG(i, res_k[i]);
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& /*x*/,
                                  const ControlType& /*u*/,
                                  bool using_scale) const override {
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    if (k >= effect_index_) return res;
    const auto& res_k = updated_results_[k].divided_gs;
    for (int i = 0; i < res_k.size(); ++i) {
      res.AddSubG(i, res_k[i]);
    }
    if (!using_scale) {
      res.VecDiv(cascade_gains_);
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& /*x*/,
                   const ControlType& /*u*/) const override {
    if (k >= effect_index_) return 0.0;
    return updated_results_[k].g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
               DGDxType* dgdx) const override {
    if (k >= effect_index_) return;
    *dgdx += updated_results_[k].dg_dx;
  }

  void AddDGDu(int k, const StateType& x, const ControlType& u,
               DGDuType* dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& /*x*/, const ControlType& /*u*/,
                  DDGDxDxType* ddgdxdx) const override {
    if (k >= effect_index_) return;
    *ddgdxdx += updated_results_[k].d2g_dx2;
  }

  void AddDDGDuDx(int k, const StateType& x, const ControlType& u,
                  DDGDuDxType* ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType& x, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              bool value_only) override {
    if (value_only) {
      for (int k = 0; k < effect_index_; ++k) {
        const StateType& state = PROB::GetStateAtStep(xs, k);
        UpdateOneStep(state, &updated_results_[k]);
      }
    } else {
      for (int k = 0; k < effect_index_; ++k) {
        const StateType& state = PROB::GetStateAtStep(xs, k);
        updated_results_[k] = UpdateOneStepWithDerivative(state);
      }
    }
  }

 private:
  struct OneStepResult {
    std::vector<double> divided_gs;
    double g = 0.0;
    DGDxType dg_dx;
    DGDuType dg_du;
    DDGDxDxType d2g_dx2;
    DDGDuDuType d2g_du2;
    DDGDuDxType d2g_du_dx;

    void SetZero() {
      std::fill(divided_gs.begin(), divided_gs.end(), 0.0);
      g = 0.0;
      dg_dx.setZero();
      dg_du.setZero();
      d2g_dx2.setZero();
      d2g_du2.setZero();
      d2g_du_dx.setZero();
    }
  };

  OneStepResult UpdateOneStepWithDerivative(const StateType& state) {
    const double x = state[PROB::kStateXIndex];
    const double y = state[PROB::kStateYIndex];
    const double theta = state[PROB::kStateThetaIndex];
    const double sin_theta =
        enable_fast_math_ ? fast_math::Sin(theta) : std::sin(theta);
    const double cos_theta =
        enable_fast_math_ ? fast_math::Cos(theta) : std::cos(theta);

    OneStepResult result;
    result.SetZero();
    result.divided_gs.resize(sub_names_.size(), 0.0);

    std::vector<double>* divided_g = &result.divided_gs;
    double* p_g = &result.g;
    double* p_dg_dx = &result.dg_dx(PROB::kStateXIndex);
    double* p_dg_dy = &result.dg_dx(PROB::kStateYIndex);
    double* p_dg_dtheta = &result.dg_dx(PROB::kStateThetaIndex);

    double* p_d2g_dx2 = &result.d2g_dx2(PROB::kStateXIndex, PROB::kStateXIndex);
    double* p_d2g_dy2 = &result.d2g_dx2(PROB::kStateYIndex, PROB::kStateYIndex);
    double* p_d2g_dtheta2 =
        &result.d2g_dx2(PROB::kStateThetaIndex, PROB::kStateThetaIndex);
    double* p_d2g_dx_dy =
        &result.d2g_dx2(PROB::kStateXIndex, PROB::kStateYIndex);
    double* p_d2g_dx_dtheta =
        &result.d2g_dx2(PROB::kStateXIndex, PROB::kStateThetaIndex);
    double* p_d2g_dy_dtheta =
        &result.d2g_dx2(PROB::kStateYIndex, PROB::kStateThetaIndex);

    for (int i = 0; i < circle_center_offsets_.size(); ++i) {
      const Vec2d& offset = circle_center_offsets_[i];
      AppendOneCircleCostWithDerivative(
          x, y, sin_theta, cos_theta, /*forward_shift=*/offset.x(),
          /*rightward_shift=*/-offset.y(), circle_radiuses_[i],
          cascade_buffers_, cascade_gains_final_, divided_g, p_g, p_dg_dx,
          p_dg_dy, p_dg_dtheta, p_d2g_dx2, p_d2g_dy2, p_d2g_dtheta2,
          p_d2g_dx_dy, p_d2g_dx_dtheta, p_d2g_dy_dtheta);
    }

    result.d2g_dx2(PROB::kStateYIndex, PROB::kStateXIndex) =
        result.d2g_dx2(PROB::kStateXIndex, PROB::kStateYIndex);
    result.d2g_dx2(PROB::kStateThetaIndex, PROB::kStateXIndex) =
        result.d2g_dx2(PROB::kStateXIndex, PROB::kStateThetaIndex);
    result.d2g_dx2(PROB::kStateThetaIndex, PROB::kStateYIndex) =
        result.d2g_dx2(PROB::kStateYIndex, PROB::kStateThetaIndex);
    return result;
  }

  void UpdateOneStep(const StateType& state, OneStepResult* res) {
    const double x = state[PROB::kStateXIndex];
    const double y = state[PROB::kStateYIndex];
    const double theta = state[PROB::kStateThetaIndex];
    const double sin_theta =
        enable_fast_math_ ? fast_math::Sin(theta) : std::sin(theta);
    const double cos_theta =
        enable_fast_math_ ? fast_math::Cos(theta) : std::cos(theta);

    res->divided_gs.resize(sub_names_.size(), 0.0);
    std::fill(res->divided_gs.begin(), res->divided_gs.end(), 0.0);
    res->g = 0.0;
    for (int i = 0; i < circle_center_offsets_.size(); ++i) {
      const Vec2d& offset = circle_center_offsets_[i];
      AppendOneCircleCost(x, y, sin_theta, cos_theta,
                          /*forward_shift=*/offset.x(),
                          /*rightward_shift=*/-offset.y(), circle_radiuses_[i],
                          cascade_buffers_, cascade_gains_final_,
                          &res->divided_gs, &res->g);
    }
  }

  void AppendOneCircleCostWithDerivative(
      double x, double y, double sin_theta, double cos_theta,
      double forward_shift, double rightward_shift, double radius,
      const std::vector<double>& buffer_list,
      const std::vector<double>& gain_list, std::vector<double>* divided_pg,
      double* pg, double* dg_dx, double* dg_dy, double* dg_dtheta,
      double* d2g_dx2, double* d2g_dy2, double* d2g_dtheta2, double* d2g_dx_dy,
      double* d2g_dx_dtheta, double* d2g_dy_dtheta) const {
    DCHECK(divided_pg != nullptr);
    DCHECK(pg != nullptr);
    DCHECK(dg_dx != nullptr);
    DCHECK(dg_dy != nullptr);
    DCHECK(dg_dtheta != nullptr);
    DCHECK(d2g_dx2 != nullptr);
    DCHECK(d2g_dy2 != nullptr);
    DCHECK(d2g_dtheta2 != nullptr);
    DCHECK(d2g_dx_dy != nullptr);
    DCHECK(d2g_dx_dtheta != nullptr);
    DCHECK(d2g_dy_dtheta != nullptr);

    MinSegmentDistanceProblem::SecondOrderDerivativeType der;
    const double xr =
        x + forward_shift * cos_theta + rightward_shift * sin_theta;
    const double yr =
        y + forward_shift * sin_theta - rightward_shift * cos_theta;
    const double min_dis =
        curb_msd_.EvaluateWithSecondOrderDerivatives(Vec2d{xr, yr}, &der);

    for (int i = 0; i < buffer_list.size(); ++i) {
      const double gain = gain_list[i];

      const double invasion = buffer_list[i] + radius - min_dis;
      if (invasion <= 0.0) {
        continue;
      }
      const double delta_y = y - yr;
      const double delta_x = x - xr;
      const double dmsd_dx = der.df_dx;
      const double dmsd_dy = der.df_dy;
      const double dmsd_dtheta = delta_y * der.df_dx - delta_x * der.df_dy;
      const double d2msd_dx_dx = der.d2f_dx_dx;
      const double d2msd_dx_dy = der.d2f_dx_dy;
      const double d2msd_dx_dtheta =
          delta_y * der.d2f_dx_dx - delta_x * der.d2f_dx_dy;
      const double d2msd_dy_dy = der.d2f_dy_dy;
      const double d2msd_dy_dtheta =
          delta_y * der.d2f_dx_dy - delta_x * der.d2f_dy_dy;
      const double d2msd_dtheta_dtheta =
          delta_y * (delta_y * der.d2f_dx_dx - delta_x * der.d2f_dx_dy) +
          delta_y * der.df_dy + delta_x * der.df_dx -
          delta_x * (delta_y * der.d2f_dx_dy - delta_x * der.d2f_dy_dy);

      const double g = gain * invasion * invasion;
      const double dg_dmsd = -2.0 * invasion * gain;

      (*divided_pg)[i] += g;
      *pg += g;
      *dg_dx += dg_dmsd * dmsd_dx;
      *dg_dy += dg_dmsd * dmsd_dy;
      *dg_dtheta += dg_dmsd * dmsd_dtheta;
      *d2g_dx2 += dg_dmsd * d2msd_dx_dx + 2.0 * gain * dmsd_dx * dmsd_dx;
      *d2g_dy2 += dg_dmsd * d2msd_dy_dy + 2.0 * gain * dmsd_dy * dmsd_dy;
      *d2g_dtheta2 += dg_dmsd * d2msd_dtheta_dtheta +
                      2.0 * gain * dmsd_dtheta * dmsd_dtheta;
      *d2g_dx_dy += dg_dmsd * d2msd_dx_dy + 2.0 * gain * dmsd_dx * dmsd_dy;
      *d2g_dx_dtheta +=
          dg_dmsd * d2msd_dx_dtheta + 2.0 * gain * dmsd_dx * dmsd_dtheta;
      *d2g_dy_dtheta +=
          dg_dmsd * d2msd_dy_dtheta + 2.0 * gain * dmsd_dy * dmsd_dtheta;
    }
  }

  void AppendOneCircleCost(double x, double y, double sin_theta,
                           double cos_theta, double forward_shift,
                           double rightward_shift, double radius,
                           const std::vector<double>& buffer_list,
                           const std::vector<double>& gain_list,
                           std::vector<double>* divided_pg, double* pg) const {
    DCHECK(divided_pg != nullptr);
    DCHECK(pg != nullptr);
    const double xr =
        x + forward_shift * cos_theta + rightward_shift * sin_theta;
    const double yr =
        y + forward_shift * sin_theta - rightward_shift * cos_theta;
    const double min_dis = curb_msd_.Evaluate(Vec2d{xr, yr});

    for (int i = 0; i < buffer_list.size(); ++i) {
      const double invasion = radius + buffer_list[i] - min_dis;
      if (invasion <= 0.0) {
        continue;
      }
      const double g = gain_list[i] * invasion * invasion;
      (*divided_pg)[i] += g;
      *pg += g;
    }
  }

  VehicleGeometryParamsProto vehicle_geometry_params_;
  MinSegmentDistanceProblem curb_msd_;

  std::vector<double> cascade_buffers_;
  const std::vector<double> cascade_gains_;
  std::vector<Vec2d> circle_center_offsets_;
  std::vector<double> circle_radiuses_;
  std::vector<double> cascade_gains_final_;
  std::vector<std::string> sub_names_;

  bool enable_fast_math_ = false;

  int effect_index_;

  std::array<OneStepResult, PROB::kHorizon> updated_results_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MSD_STATIC_BOUNDARY_COST_H_
