#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/auto_tuning_common_flags.h"
#include "offboard/planner/ml/params_tuning/dopt_auto_tuning/proto/auto_tuning.pb.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/util.h"
#include "onboard/planner/optimization/ddp/ddp_cost_manager_hook.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer_hook.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/trajectory_point.h"

DECLARE_int32(planner_dopt_canvas_level);
DECLARE_bool(planner_dopt_symmetrize_j_hessian);
DECLARE_bool(enable_stepsize_after_line_search);

namespace qcraft {
namespace planner {

#define DDPVLOG(verboselevel) \
  VLOG_IF(verboselevel, ((verbosity_ >= verboselevel)))

// Type PROB is the DDP problem definition.
// A DDP problem is a tuple (f, g^n) where f is the (non-linear) system dynamics
// and g^n is the (time-variable) cost, both of which functions of x and u.
template <typename PROB>
class DdpOptimizer {
 public:
  static constexpr int kStateSize = PROB::kStateSize;
  static constexpr int kControlSize = PROB::kControlSize;
  static constexpr int kHorizon = PROB::kHorizon;

  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;
  using MfobDAT = MixedFourthOrderBicycle<kDdpTrajectoryStepsDAT>;

  // problem will not be owned.
  DdpOptimizer(const PROB *problem, std::string owner, int verbosity,
               DdpOptimizerParamsProto params, std::string dopt_tag);

  DdpOptimizer(const PROB *problem, std::string owner, int verbosity,
               DdpOptimizerParamsProto params)
      : DdpOptimizer(problem, owner, verbosity, params, /*dopt_tag=*/"0") {}

  DdpOptimizer(const PROB *problem, std::string owner, int verbosity)
      : DdpOptimizer(problem, owner, verbosity,
                     /*params=*/CreateDefaultParams()) {}

  DdpOptimizer(const PROB *problem, std::string owner)
      : DdpOptimizer(problem, owner, /*verbosity=*/0) {}

  explicit DdpOptimizer(const PROB *problem)
      : DdpOptimizer(problem, /*owner=*/"") {}

  using HookType = typename qcraft::planner::DdpOptimizerHook<PROB>;

  // Not owned.
  void AddHook(HookType *hook) { hooks_.push_back(hook); }

  void SetInitialPoints(std::vector<TrajectoryPoint> init_points) {
    QCHECK_EQ(init_points.size(), kHorizon);
    init_points_ = std::move(init_points);
    for (int k = 0; k < kHorizon; ++k) {
      init_points_[k].set_t(k * problem_->dt());
    }
    init_points_.front().set_theta(
        init_points_[1].theta() +
        NormalizeAngle(init_points_.front().theta() - init_points_[1].theta()));
  }

  absl::StatusOr<std::vector<TrajectoryPoint>> Solve(
      bool forward, bool enable_iteration_failure_postprocess = true);

  // This function will compute the accumulative discounted(based on gamma) cost
  // for different cost type, and store them separately in
  // AccumulatedDiscountedCostsProto.
  void EvaluateEachDiscountedAccumulativeCost(
      const MfobDAT::StatesType &xs, const MfobDAT::ControlsType &us,
      double gamma, AccumulatedDiscountedCostsProto *costs) const;

  int problem_costs_size() const { return problem_->costs().size(); }

 protected:
  using FType = typename PROB::FType;
  using DFDxType = typename PROB::DFDxType;
  using DFDuType = typename PROB::DFDuType;
  using DDFDxDxType = typename PROB::DDFDxDxType;
  using DDFDxDuType = typename PROB::DDFDxDuType;
  using DDFDuDxType = typename PROB::DDFDuDxType;
  using DDFDuDuType = typename PROB::DDFDuDuType;
  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;
  using ClampInfo = typename PROB::ClampInfo;

  // Returns the optimal total cost, i.e. Js[0].
  double SolveLinearDdp(const PROB &problem, const StatesType &xs,
                        const ControlsType &us, double curr_cost,
                        StatesType *dxs, ControlsType *dus,
                        ControlsType *dus_open,
                        std::array<DDGDuDxType, kHorizon> *dus_close_gain,
                        int *k_ne);

  template <typename HESSG>
  HESSG GradJDot(const Eigen::Matrix<double, 1, kStateSize> &grad_j,
                 const std::array<HESSG, kStateSize> &hess_f) {
    HESSG hess_g = HESSG::Zero();
    for (int i = 0; i < kStateSize; ++i) {
      hess_g += grad_j[i] * hess_f[i];
    }
    return hess_g;
  }

  double EvaluateCost(const StatesType &xs, const ControlsType &us,
                      absl::flat_hash_map<std::string, double> *cost_map) const;
  double EvaluateCost(const StatesType &xs, const ControlsType &us) const {
    return EvaluateCost(xs, us, /*cost_map=*/nullptr);
  }

  double LineSearchAndEvaluateCost(int iteration,
                                   const StatesType &tentative_xs,
                                   const ControlsType &tentative_us,
                                   const StatesType &full_dxs,
                                   const ControlsType &full_dus, double alpha,
                                   typename HookType::OptimizerInspector *oi);

  double StepSizeAjustmentAndEvaluateCost(
      int iteration, const StatesType &xs, const ControlsType &us,
      int k_stepsize, typename HookType::OptimizerInspector *oi);

  ControlsType OptimizeInitialControl(const StatesType &init_xs,
                                      const ControlsType &init_us);

  DdpOptimizerParamsProto CreateDefaultParams() {
    DdpOptimizerParamsProto params;
    return params;
  }

  void MaybeDrawDdpIterCanvas(int iteration, StatesType xs, ControlsType us);

 private:
  const PROB *problem_;
  std::string owner_;
  int verbosity_;
  DdpOptimizerParamsProto params_;
  std::string dopt_tag_;  // Delete after DoptPlanner is deprecated.
  std::unique_ptr<DdpCostManagerHook<PROB>> cost_manager_hook_;

  std::vector<HookType *> hooks_;
  std::vector<TrajectoryPoint> init_points_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementations.
template <typename PROB>
DdpOptimizer<PROB>::DdpOptimizer(const PROB *problem, std::string owner,
                                 int verbosity, DdpOptimizerParamsProto params,
                                 std::string dopt_tag)
    : problem_(CHECK_NOTNULL(problem)),
      owner_(std::move(owner)),
      verbosity_(verbosity),
      params_(std::move(params)),
      dopt_tag_(std::move(dopt_tag)) {
  cost_manager_hook_ = std::make_unique<DdpCostManagerHook<PROB>>();
  for (const auto &helper : problem_->cost_helpers()) {
    cost_manager_hook_->AddCostHelper(helper.get());
  }
  for (const auto &cost : problem_->costs()) {
    cost_manager_hook_->AddCost(cost.get());
  }
  AddHook(cost_manager_hook_.get());
}

template <typename PROB>
double DdpOptimizer<PROB>::SolveLinearDdp(
    const PROB &problem, const StatesType &xs, const ControlsType &us,
    double curr_cost, StatesType *dxs, ControlsType *dus,
    ControlsType *dus_open, std::array<DDGDuDxType, kHorizon> *dus_close_gain,
    int *k_ne) {
  SCOPED_QTRACE("DdpOptimizer::SolveLinearDdp");
  const StateType x0 = PROB::GetStateAtStep(xs, 0);

  using JType = double;
  using DJDxType = Eigen::Matrix<double, 1, kStateSize>;
  using DDJDxDxType = Eigen::Matrix<double, kStateSize, kStateSize>;

  std::array<JType, kHorizon + 1> Js{{0}};
  std::array<DJDxType, kHorizon + 1> dJdxs;
  std::array<DDJDxDxType, kHorizon + 1> ddJdxdxs;

  std::array<DDGDuDuType, kHorizon> As;
  std::array<DDGDuDuType, kHorizon> Ainvs;
  std::array<DGDuType, kHorizon> b_bases;
  std::array<DDGDuDxType, kHorizon> b_lins;
  std::array<GType, kHorizon> c_bases{{0}};

  // Last step has no J_{k+1}.
  Js[kHorizon] = 0.0;
  dJdxs[kHorizon] = DJDxType::Zero();
  ddJdxdxs[kHorizon] = DDJDxDxType::Zero();

  // Step_size adjusment method Ne
  std::optional<int> k_n_e;
  constexpr double kEps = 1e-8;

  // Backward pass: recursively evaluate J_k.
  DDPVLOG(3) << "Problem evaluating f and g (and derivatives)";
  std::array<typename PROB::FDerivatives, kHorizon> fds;
  std::array<typename PROB::GDerivatives, kHorizon> gds;
  problem.EvaluateFDerivativesForAllSteps(xs, us, &fds);
  problem.AddGDerivativesForAllSteps(xs, us, &gds);
  for (int k = kHorizon - 1; k >= 0; --k) {
    const FType &f = fds[k].value;
    const DFDxType &dfdx = fds[k].dfdx;
    const DFDuType &dfdu = fds[k].dfdu;
    const DDFDxDxType &ddfdxdx = fds[k].ddfdxdx;
    const DDFDuDxType &ddfdudx = fds[k].ddfdudx;
    const DDFDuDuType &ddfdudu = fds[k].ddfdudu;

    const GType &g = gds[k].value;
    const DGDxType &dgdx = gds[k].dgdx;
    const DGDuType &dgdu = gds[k].dgdu;
    const DDGDxDxType &ddgdxdx = gds[k].ddgdxdx;
    const DDGDuDxType &ddgdudx = gds[k].ddgdudx;
    const DDGDuDuType &ddgdudu = gds[k].ddgdudu;

    const JType &J = Js[k + 1];
    const DJDxType &dJdx = dJdxs[k + 1];
    const DDJDxDxType &ddJdxdx = ddJdxdxs[k + 1];

    As[k] =
        ddgdudu + GradJDot(dJdx, ddfdudu) + dfdu.transpose() * ddJdxdx * dfdu;
    Ainvs[k] = As[k].inverse();
    b_bases[k] = dgdu + dJdx * dfdu;
    b_lins[k] =
        ddgdudx + GradJDot(dJdx, ddfdudx) + dfdu.transpose() * ddJdxdx * dfdx;
    c_bases[k] = g + J;

    const double dJ_expected =
        0.5 * b_bases[k] * Ainvs[k] * b_bases[k].transpose();
    if (dJ_expected > kEps && !k_n_e.has_value()) k_n_e = k + 1;

    Js[k] = c_bases[k] - dJ_expected;
    dJdxs[k] = dgdx + dJdx * dfdx - b_bases[k] * Ainvs[k] * b_lins[k];
    ddJdxdxs[k] = ddgdxdx + GradJDot(dJdx, ddfdxdx) +
                  dfdx.transpose() * ddJdxdx * dfdx -
                  b_lins[k].transpose() * Ainvs[k] * b_lins[k];
    if (FLAGS_planner_dopt_symmetrize_j_hessian) {
      // Symmetrize J hessian. If we don't manually do this, numerical error
      // may accumulate over backward steps and may come to dominate A in the
      // late steps (early time steps in the forward direction), making A^-1
      // significantly inaccurate.
      ddJdxdxs[k] = (ddJdxdxs[k] + ddJdxdxs[k].transpose()) * 0.5;
    }

    if (UNLIKELY(VLOG_IS_ON(4))) {
      DDPVLOG(4) << "---------------------------------- Backward pass step "
                 << k << " ----------------------------------";

      DDPVLOG(4) << "f = " << f.transpose();
      DDPVLOG(4) << "dfdx = " << std::endl << dfdx;
      DDPVLOG(4) << "dfdu = " << std::endl << dfdu;

      DDPVLOG(4) << "g = " << g;
      DDPVLOG(4) << "dgdx = " << dgdx;
      DDPVLOG(4) << "dgdu = " << dgdu;
      DDPVLOG(4) << "ddgdxdx = " << std::endl << ddgdxdx;
      DDPVLOG(4) << "ddgdudx = " << std::endl << ddgdudx;
      DDPVLOG(4) << "ddgdudu = " << std::endl << ddgdudu;

      DDPVLOG(4) << "A = " << std::endl << As[k];
      DDPVLOG(4) << "Ainv = " << std::endl << Ainvs[k];
      DDPVLOG(4) << "b_base = " << b_bases[k];
      DDPVLOG(4) << "b_lin = " << std::endl << b_lins[k];
      DDPVLOG(4) << "c = " << c_bases[k];

      DDPVLOG(4) << "J = " << Js[k];
      DDPVLOG(4) << "dJdx = " << dJdxs[k];
      DDPVLOG(4) << "ddJdxdx = " << std::endl << ddJdxdxs[k];
    }
  }

  // Forward pass: evaluate u_k^* and x_k^*.
  *dxs = StatesType::Zero();
  *dus = ControlsType::Zero();

  const int k_stepsize_upper = k_n_e.has_value() ? *k_n_e : 0;
  int k_stepsize = -k_stepsize_upper;
  constexpr double kDuLimit = 1e4;
  do {
    k_stepsize = (k_stepsize + k_stepsize_upper) >> 1;
    StateType x = x0;
    DDPVLOG(4) << "---------------------------------- Step-size adjustment "
               << k_stepsize << " ----------------------------------";
    for (int k = 0; k < kHorizon; ++k) {
      const StateType dx = x - PROB::GetStateAtStep(xs, k);
      PROB::SetStateAtStep(dx, k, dxs);
      if (k < k_stepsize) {
        x = PROB::GetStateAtStep(xs, k + 1);
        (*dus_close_gain)[k] = DDGDuDxType::Zero();
        PROB::SetControlAtStep(ControlType::Zero(), k, dus);
        PROB::SetControlAtStep(ControlType::Zero(), k, dus_open);
        if (UNLIKELY(VLOG_IS_ON(4))) {
          DDPVLOG(4) << "---------------------------------- Forward pass "
                        "adjusted  step "
                     << k << " ----------------------------------";
          DDPVLOG(4) << "x = " << x.transpose();
          DDPVLOG(4) << "u = " << (PROB::GetControlAtStep(us, k)).transpose();
        }
      } else {
        const DDGDuDuType &Ainv = Ainvs[k];
        const DGDuType &b_base = b_bases[k];
        const DDGDuDxType &b_lin = b_lins[k];
        const ControlType du_open = -Ainv * b_base.transpose();
        const DDGDuDxType du_close_gain = -Ainv * b_lin;
        const ControlType du = du_open + du_close_gain * dx;
        const ControlType u = PROB::GetControlAtStep(us, k) + du;
        x = problem.EvaluateF(k, x, u);
        (*dus_close_gain)[k] = du_close_gain;
        PROB::SetControlAtStep(du, k, dus);
        PROB::SetControlAtStep(du_open, k, dus_open);
        if (UNLIKELY(VLOG_IS_ON(4))) {
          DDPVLOG(4) << "---------------------------------- Forward pass step "
                     << k << " ----------------------------------";
          DDPVLOG(4) << "dx = " << dx.transpose();
          DDPVLOG(4) << "x = " << x.transpose();
          DDPVLOG(4) << "du = " << du.transpose();
          DDPVLOG(4) << "u = " << u.transpose();
        }
      }
    }
  } while ((dus->maxCoeff() > kDuLimit || dus->minCoeff() < -kDuLimit) &&
           k_stepsize < (k_stepsize_upper - 1));

  *k_n_e = k_stepsize_upper;

  // Record QEvent when trigger step-size adjusment.
  if (k_stepsize > 0 && k_n_e.has_value()) {
    if (owner_ == "trajectory_optimizer") {
      QEVENT_EVERY_N_SECONDS(
          "runbing", owner_ + "_step_size_adjustment", 0.2,
          [&](QEvent *qevent) { qevent->AddField("k_stepsize", k_stepsize); });
    }
  }

  // Record Qevent if unreasonable du appeared.
  constexpr double kDusNormCheckLimit = 200.0;
  if (dus->norm() > kDusNormCheckLimit && owner_ == "trajectory_optimizer") {
    QEVENT_EVERY_N_SECONDS(
        "runbing", "traj_opt_unreasonable_du", 0.2,
        [&](QEvent *qevent) { qevent->AddField("dus", dus->norm()); });
  }

  // Record QEvent when has no-positive quu.
  bool have_quu_nonpositive = false;
  int quu_nonpositive_index = -1;
  if (owner_ == "trajectory_optimizer") {
    for (int k = 0; k < kHorizon; ++k) {
      Eigen::EigenSolver<DDGDuDuType> es(As[k]);
      if (es.pseudoEigenvalueMatrix().minCoeff() < 0.0) {
        QEVENT_EVERY_N_SECONDS(
            "runbing", "traj_opt_quu_nopositive", 5.0, [&](QEvent *qevent) {
              qevent->AddField("index", k)
                  .AddField("value", es.pseudoEigenvalueMatrix().minCoeff());
            });
        have_quu_nonpositive = true;
        quu_nonpositive_index = k;
        break;
      }
    }
  }

  // Record QEvent if Js0 > current cost, record quu no-positive and
  // unreasonable at the same time.
  constexpr double kJs0DropThreshold = 0.1;
  if (Js[0] > (curr_cost + kJs0DropThreshold) &&
      owner_ == "trajectory_optimizer") {
    QEVENT_EVERY_N_SECONDS(
        "runbing", "traj_opt_drop_negative", 0.2, [&](QEvent *qevent) {
          qevent->AddField("Js0", Js[0])
              .AddField("curr_cost", curr_cost)
              .AddField("quu_nonpositive", have_quu_nonpositive)
              .AddField("quu_nonpositive_index", quu_nonpositive_index)
              .AddField("dus_norm", dus->norm());
        });
  }

  return Js[0];
}

template <typename PROB>
void DdpOptimizer<PROB>::MaybeDrawDdpIterCanvas(int iteration, StatesType xs,
                                                ControlsType us) {
  if (FLAGS_planner_dopt_canvas_level >= 3) {
    // pc for per-cost channels; pi for per-iteration channels.
    std::vector<vis::Canvas *> canvas_cost_grad_pc_x;
    std::vector<vis::Canvas *> canvas_cost_grad_pc_u;
    std::vector<vis::Canvas *> canvas_cost_grad_pi_x;
    std::vector<vis::Canvas *> canvas_cost_grad_pi_u;
    for (const auto &cost : problem_->costs()) {
      canvas_cost_grad_pc_x.push_back(&vantage_client_man::GetCanvas(
          absl::StrFormat("dopt/%s/grad/%s/iter_%03d/x", dopt_tag_,
                          cost->name(), iteration)));
      canvas_cost_grad_pc_u.push_back(&vantage_client_man::GetCanvas(
          absl::StrFormat("dopt/%s/grad/%s/iter_%03d/u", dopt_tag_,
                          cost->name(), iteration)));
      canvas_cost_grad_pi_x.push_back(&vantage_client_man::GetCanvas(
          absl::StrFormat("dopt/%s/grad_iters/iter_%03d/%s/x", dopt_tag_,
                          iteration, cost->name())));
      canvas_cost_grad_pi_u.push_back(&vantage_client_man::GetCanvas(
          absl::StrFormat("dopt/%s/grad_iters/iter_%03d/%s/u", dopt_tag_,
                          iteration, cost->name())));
    }
    vis::Canvas *canvas_total_cost_grad_pc_x =
        &vantage_client_man::GetCanvas(absl::StrFormat(
            "dopt/%s/grad/total/iter_%03d/x", dopt_tag_, iteration));
    vis::Canvas *canvas_total_cost_grad_pc_u =
        &vantage_client_man::GetCanvas(absl::StrFormat(
            "dopt/%s/grad/total/iter_%03d/u", dopt_tag_, iteration));
    vis::Canvas *canvas_total_cost_grad_pi_x =
        &vantage_client_man::GetCanvas(absl::StrFormat(
            "dopt/%s/grad_iters/iter_%03d/total/x", dopt_tag_, iteration));
    vis::Canvas *canvas_total_cost_grad_pi_u =
        &vantage_client_man::GetCanvas(absl::StrFormat(
            "dopt/%s/grad_iters/iter_%03d/total/u", dopt_tag_, iteration));
    for (int k = 0; k < PROB::kHorizon; ++k) {
      const typename PROB::StateType x = PROB::GetStateAtStep(xs, k);
      const typename PROB::ControlType u = PROB::GetControlAtStep(us, k);
      const Vec2d pos = PROB::StateGetPos(x);
      const double z =
          k * problem_->dt() * kSpaceTimeVisualizationDefaultTimeScale;

      Vec2d total_grad_state_xy = Vec2d::Zero();
      Vec2d total_grad_control_xy = Vec2d::Zero();
      for (int j = 0; j < problem_->costs().size(); ++j) {
        const auto &cost = problem_->costs()[j];
        const typename PROB::DGDxType grad_x = cost->EvaluateDGDx(k, x, u);
        const typename PROB::DGDuType grad_u = cost->EvaluateDGDu(k, x, u);
        //////////////////////////////////////////////////////////////////////
        // Code specific to SOB and TOB problems.
        const Vec2d grad_state_xy(grad_x[0], grad_x[1]);
        const Vec2d grad_control_xy(grad_u[0], grad_u[1]);
        //////////////////////////////////////////////////////////////////////
        if (FLAGS_planner_dopt_canvas_level >= 4) {
          canvas_cost_grad_pc_x[j]->SetGroundZero(1);
          canvas_cost_grad_pc_x[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_state_xy, z),
                                             vis::Color(0.8, 0.6, 0.2));
          canvas_cost_grad_pc_u[j]->SetGroundZero(1);
          canvas_cost_grad_pc_u[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_control_xy, z),
                                             vis::Color(0.2, 0.6, 0.8));
          canvas_cost_grad_pi_x[j]->SetGroundZero(1);
          canvas_cost_grad_pi_x[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_state_xy, z),
                                             vis::Color(0.8, 0.6, 0.2));
          canvas_cost_grad_pi_u[j]->SetGroundZero(1);
          canvas_cost_grad_pi_u[j]->DrawLine(Vec3d(pos, z),
                                             Vec3d(pos + grad_control_xy, z),
                                             vis::Color(0.2, 0.6, 0.8));
        }
        total_grad_state_xy += grad_state_xy;
        total_grad_control_xy += grad_control_xy;
      }
      canvas_total_cost_grad_pc_x->SetGroundZero(1);
      canvas_total_cost_grad_pc_x->DrawLine(Vec3d(pos, z),
                                            Vec3d(pos + total_grad_state_xy, z),
                                            vis::Color(0.9, 0.7, 0.3));
      canvas_total_cost_grad_pc_u->SetGroundZero(1);
      canvas_total_cost_grad_pc_u->DrawLine(
          Vec3d(pos, z), Vec3d(pos + total_grad_control_xy, z),
          vis::Color(0.3, 0.7, 0.9));
      canvas_total_cost_grad_pi_x->SetGroundZero(1);
      canvas_total_cost_grad_pi_x->DrawLine(Vec3d(pos, z),
                                            Vec3d(pos + total_grad_state_xy, z),
                                            vis::Color(0.9, 0.7, 0.3));
      canvas_total_cost_grad_pi_u->SetGroundZero(1);
      canvas_total_cost_grad_pi_u->DrawLine(
          Vec3d(pos, z), Vec3d(pos + total_grad_control_xy, z),
          vis::Color(0.3, 0.7, 0.9));
    }
  }
}

template <typename PROB>
absl::StatusOr<std::vector<TrajectoryPoint>> DdpOptimizer<PROB>::Solve(
    bool forward, bool enable_iteration_failure_postprocess) {
  ScopedMultiTimer timer("dopt");
  SCOPED_QTRACE_ARG1("DdpOptimizer::Solve", "num_costs",
                     problem_->costs().size());

  if (VLOG_IS_ON(3)) {
    for (int k = 0; k < kHorizon; ++k) {
      DDPVLOG(3) << "init_points[" << k
                 << "]: " << init_points_[k].DebugString();
    }
  }

  DDPVLOG(1) << "Generating initial controls and initial states";
  // We do not rollout initial states but use fitted initial states.
  const StateType x0 = problem_->FitInitialState(init_points_);
  const ControlsType init_us = problem_->FitControl(init_points_, x0);
  const StatesType init_xs = problem_->FitState(init_points_);

  DDPVLOG(3) << "init xs = " << init_xs.transpose();
  DDPVLOG(3) << "init us = " << init_us.transpose();
  timer.Mark("init control roll out");

  DDPVLOG(1) << "Solve starts";
  typename HookType::OptimizerInspector oi;
  StatesType xs = init_xs;
  ControlsType us = init_us;
  for (auto *hook : hooks_) {
    hook->OnSolveStart(xs, us, oi);
  }
  timer.Mark("OnSolveStart");

  double total_cost = 0.0;
  {
    const double init_cost = EvaluateCost(init_xs, init_us, &oi.cost_map);
    if (VLOG_IS_ON(2)) {
      DDPVLOG(2) << "Total cost: " << init_cost;
      for (const auto &[cost_name, cost] : oi.cost_map) {
        DDPVLOG(2) << "  [" << cost_name << "] cost: " << cost;
      }
    }
    DDPVLOG(1) << "Initial cost: " << init_cost;
    total_cost = init_cost;
    oi.cost = init_cost;
  }
  timer.Mark("init EvaluateCost");

  // Set line search alphas varibles.
  // Slphas vector.
  std::vector<double> line_search_alphas;
  constexpr double kLineSearchAlphaMultiplier = 0.5;
  // Line search must start from alpha = 1.0 to make sure ine search loop will
  // Try full step du update.
  line_search_alphas.push_back(1.0);
  while (line_search_alphas.back() > params_.line_search_min_alpha()) {
    line_search_alphas.push_back(line_search_alphas.back() *
                                 kLineSearchAlphaMultiplier);
  }
  // Start alpha index in alpha vector, set to 1 because full step will be tried
  // when init.
  int alpha_idx = 1;
  const int alpha_count = line_search_alphas.size();
  int line_search_count = 0;

  int iteration;
  for (iteration = 0; iteration < params_.max_iters(); ++iteration) {
    ScopedMultiTimer iter_timer(absl::StrFormat("iter%03d", iteration));
    SCOPED_QTRACE_ARG1("ddp_iter", "iter", iteration);
    DDPVLOG(2) << "Iteration " << iteration << " starts";

    {
      SCOPED_QTRACE("iter_start");
      for (auto *hook : hooks_) {
        hook->OnIterationStart(iteration, xs, us, oi);
      }
    }

    MaybeDrawDdpIterCanvas(iteration, xs, us);

    iter_timer.Mark("OnIterationStart");

    int k_n_e = kHorizon;
    StatesType dxs = StatesType::Zero();
    ControlsType dus = ControlsType::Zero();
    ControlsType dus_open = ControlsType::Zero();
    std::array<DDGDuDxType, kHorizon> dus_close_gain;
    dus_close_gain.fill(DDGDuDxType::Zero());
    const double Js0 = SolveLinearDdp(*problem_, xs, us, total_cost, &dxs, &dus,
                                      &dus_open, &dus_close_gain, &k_n_e);
    iter_timer.Mark("linear ddp solve");

    if (!problem_->CheckDu(dus, owner_) ||
        !problem_->CheckDu(dus_open, owner_)) {
      return absl::InternalError(owner_ +
                                 " ddp optimizer has unreasonable du.");
    }

    const bool dx_converged =
        dxs.squaredNorm() < Sqr(params_.convergence_tolerance_dx());
    const bool du_converged =
        dus.squaredNorm() < Sqr(params_.convergence_tolerance_du());

    double dcost = 0.0;
    if (!dx_converged && !du_converged) {
      // Line search on du. The control flow below might appear weird, but it is
      // organized so that the calls to RollOutControl() and EvaluateCost() are
      // minimized.
      constexpr double kMinAcceptableCostDrop = 1e-6;
      const StatesType line_search_full_dxs = dxs;
      const ControlsType line_search_full_dus = dus;
      const ControlsType line_search_dus_open = dus_open;
      const std::array<DDGDuDxType, kHorizon> line_search_dus_close_gain =
          dus_close_gain;

      StatesType tentative_xs = xs + dxs;
      ControlsType tentative_us = us + dus;
      const double line_search_init_cost = total_cost;
      DDPVLOG(4) << "Line search full dxs: "
                 << line_search_full_dxs.transpose();
      DDPVLOG(4) << "Line search full dus: "
                 << line_search_full_dus.transpose();

      // Setting params_.line_search_min_alpha() to 1.0 or higher will disable
      // the line search.
      // Set curr_alpha to first line_search_alphas front.
      double curr_alpha = line_search_alphas.front();
      double next_alpha = curr_alpha;
      StatesType curr_xs = tentative_xs;
      StatesType next_xs = curr_xs;
      ControlsType curr_us = tentative_us;
      ControlsType next_us = curr_us;
      double curr_cost = LineSearchAndEvaluateCost(
          iteration, curr_xs, curr_us, line_search_full_dxs,
          line_search_full_dus, curr_alpha, &oi);
      double next_cost = curr_cost;
      DDPVLOG(2) << "Line search: init cost: " << line_search_init_cost
                 << "; Js[0]: " << Js0 << " cost at full step: " << curr_cost
                 << " du step norm: " << line_search_full_dus.norm();

      // Line search is terminated if one of the following conditions is met:
      // 1. Current step cost drop > kMinAcceptableCostDrop and next step cost >
      // current step cost.
      // 2. Next step alpha_idx >= alpha_count.
      while (!(curr_cost < line_search_init_cost - kMinAcceptableCostDrop &&
               next_cost > curr_cost) &&
             alpha_idx < alpha_count) {
        curr_alpha = next_alpha;
        curr_xs = next_xs;
        curr_us = next_us;
        curr_cost = next_cost;
        next_alpha = line_search_alphas[alpha_idx];

        const ControlsType processed_dus_open =
            line_search_dus_open * next_alpha;
        StateType x = x0;
        for (int k = 0; k < kHorizon; ++k) {
          PROB::SetStateAtStep(x, k, &next_xs);
          const StateType dx = x - PROB::GetStateAtStep(xs, k);
          const ControlType du = PROB::GetControlAtStep(processed_dus_open, k) +
                                 line_search_dus_close_gain[k] * dx;
          const ControlType u = PROB::GetControlAtStep(us, k) + du;
          PROB::SetControlAtStep(u, k, &next_us);
          x = problem_->EvaluateF(k, x, u);
        }

        next_cost = LineSearchAndEvaluateCost(
            iteration, next_xs, next_us, line_search_full_dxs,
            line_search_full_dus, next_alpha, &oi);
        DDPVLOG(2) << "Line search: next_alpha: " << next_alpha
                   << " next_cost: " << next_cost
                   << " curr_alpha: " << curr_alpha
                   << " curr_cost: " << curr_cost;
        ++alpha_idx;
        ++line_search_count;
      }

      if (next_cost < curr_cost) {
        curr_alpha = next_alpha;
        curr_xs = next_xs;
        curr_us = next_us;
        curr_cost = next_cost;
      }

      // Try step size adjustment method if line search failed.
      if (FLAGS_enable_stepsize_after_line_search &&
          curr_cost > line_search_init_cost - kMinAcceptableCostDrop) {
        int k_stepsize = -k_n_e;
        DDPVLOG(2) << "Step_size adjustment: init_cost:"
                   << line_search_init_cost
                   << "cost at full step: " << curr_cost << " k_n_e: " << k_n_e;
        do {
          curr_xs = xs;
          curr_us = us;
          k_stepsize = (k_stepsize + k_n_e) >> 1;
          StateType x = PROB::GetStateAtStep(curr_xs, k_stepsize);
          for (int k = k_stepsize; k < kHorizon; ++k) {
            PROB::SetStateAtStep(x, k, &curr_xs);
            const StateType dx = x - PROB::GetStateAtStep(xs, k);
            const ControlType du =
                PROB::GetControlAtStep(line_search_dus_open, k) +
                line_search_dus_close_gain[k] * dx;
            const ControlType u = PROB::GetControlAtStep(us, k) + du;
            PROB::SetControlAtStep(u, k, &curr_us);
            x = problem_->EvaluateF(k, x, u);
          }

          curr_cost = StepSizeAjustmentAndEvaluateCost(
              iteration, curr_xs, curr_us, k_stepsize, &oi);
          DDPVLOG(2) << "Step_size adjustment: k_stepsize: " << k_stepsize
                     << " curr_cost: " << curr_cost;
        } while ((curr_cost > line_search_init_cost - kMinAcceptableCostDrop) &&
                 k_stepsize < (k_n_e - 1));
      }

      if (params_.line_search_min_alpha() >= 1.0) {
        DDPVLOG(2) << "Line search disabled. curr_cost = " << curr_cost
                   << "; old cost = " << line_search_init_cost;
        dcost = curr_cost - total_cost;
        total_cost = curr_cost;
        tentative_xs = curr_xs;
        tentative_us = curr_us;
      } else if (curr_cost > line_search_init_cost - kMinAcceptableCostDrop) {
        // Line search terminated due to alpha limit.
        DDPVLOG(2) << "Line search failed. This iteration is rejected.";
        tentative_xs = xs;
        tentative_us = us;
      } else {
        // Acceptable new cost found. Exit line search.
        DDPVLOG(2) << "Acceptable cost drop found by line search: alpha = "
                   << curr_alpha << " cost = " << curr_cost
                   << " cost drop = " << line_search_init_cost - curr_cost
                   << " from " << line_search_init_cost;
        dcost = curr_cost - total_cost;
        total_cost = curr_cost;
        tentative_xs = curr_xs;
        tentative_us = curr_us;

        // Compute next line search start alpha.
        // Line search in next iteration should not start from first alpha, as
        // We think step length may smiliar to current step length.
        constexpr double kLineSearchDecayFactor = 2.0 / 3.0;
        const int alpha_idx_offset = -1;
        alpha_idx =
            params_.enable_adaptive_alpha()
                ? std::max(1, static_cast<int>(
                                  floor(alpha_idx * kLineSearchDecayFactor) +
                                  alpha_idx_offset))
                : 1;
      }
      // Final update.

      {
        SCOPED_QTRACE("linear_search_iter_start");
        for (auto *hook : hooks_) {
          hook->OnLineSearchIterationStart(
              iteration, tentative_xs, tentative_us, line_search_full_dxs,
              line_search_full_dus, curr_alpha, oi);
        }
      }
      oi.cost = total_cost;

      dxs = tentative_xs - xs;
      dus = tentative_us - us;
      xs = tentative_xs;
      us = tentative_us;
      DDPVLOG(3) << "End of line search : cost = " << total_cost;
      iter_timer.Mark("line search");
      // End of line search.
    }

    if (UNLIKELY(VLOG_IS_ON(3))) {
      DDPVLOG(3) << "dxs = " << dxs.transpose();
      DDPVLOG(3) << "xs = " << xs.transpose();
      DDPVLOG(3) << "dus = " << dus.transpose();
      DDPVLOG(3) << "us = " << us.transpose();
    }

    total_cost = EvaluateCost(xs, us, &oi.cost_map);
    oi.cost = total_cost;
    oi.js0 = Js0;
    if (VLOG_IS_ON(2)) {
      DDPVLOG(2) << "Total cost: " << total_cost;
      for (const auto &[cost_name, cost] : oi.cost_map) {
        DDPVLOG(2) << "  [" << cost_name << "] cost: " << cost;
      }
    }

    for (auto *hook : hooks_) {
      hook->OnIterationEnd(iteration, xs, us, oi);
    }
    iter_timer.Mark("OnIterationEnd");

    if (du_converged) {
      DDPVLOG(1) << "Terminating due to du convergence.";
      break;
    }
    if (dx_converged) {
      DDPVLOG(1) << "Terminating due to dx convergence.";
      break;
    }
    if (std::abs(dcost) < params_.convergence_tolerance_dcost()) {
      DDPVLOG(1) << "Terminating due to cost convergence.";
      constexpr double drop_failed_js0_cost = 5.0;
      if ((total_cost - Js0) > drop_failed_js0_cost && iteration == 0 &&
          owner_ == "trajectory_optimizer") {
        QEVENT("runbing", "traj_opt_ddp_drop_failed", [&](QEvent *qevent) {
          qevent->AddField("cost", total_cost).AddField("Js0", Js0);
        });
        QLOG(WARNING) << "DDP optimizer drop failed in(" << owner_ << "), Js0("
                      << Js0 << "), (total_cost(" << total_cost << ").";
      }
      break;
    }
    iter_timer.Mark("check convergence");
    // End of iteration.
  }
  timer.Mark("iterations");

  if (iteration < params_.max_iters()) {
    DDPVLOG(1) << "DDP optimizer finished in " << iteration
               << " iterations, line search count: " << line_search_count
               << ".";
  } else {
    QLOG(WARNING) << "DDP optimizer did not finish in " << iteration
                  << " iterations";
  }
  DDPVLOG(1) << "Final cost: " << total_cost;
  DDPVLOG(3) << "Final xs: " << xs.transpose();
  DDPVLOG(3) << "Final us: " << us.transpose();

  // Only post process longitudinal control and state.
  // If ddp optimizer drop failed at first iteration, refuse to postprocess
  // trajectory.
  if (iteration > 0 || enable_iteration_failure_postprocess) {
    StateType x = x0;
    for (int k = 0; k < kHorizon; ++k) {
      if ((forward && PROB::v(xs, k) < 0.0) ||
          (!forward && PROB::v(xs, k) > 0.0)) {
        problem_->SetFullStopTrajectoryFromStep(k, x, &xs, &us);
        break;
      }
      PROB::SetStateAtStep(x, k, &xs);
      ControlType u;
      ClampInfo state_clamp_info, control_clamp_info;
      u = problem_->PostProcessLonU(
          PROB::GetControlAtStep(us, k), x,
          k != kHorizon - 1
              ? PROB::GetStateAtStep(xs, k + 1)
              : problem_->EvaluateF(k, x, PROB::GetControlAtStep(us, k)),
          &control_clamp_info, forward);
      PROB::SetControlAtStep(u, k, &us);
      const auto x_origin = problem_->EvaluateF(k, x, u);
      x = problem_->PostProcessLonX(x_origin, u, &state_clamp_info);
    }
  }

  for (auto *hook : hooks_) {
    hook->OnSolveEnd(xs, us, oi);
  }
  timer.Mark("OnSolveEnd");

  std::vector<TrajectoryPoint> res;
  res.resize(kHorizon);
  for (int k = 0; k < kHorizon; ++k) {
    TrajectoryPoint &point = res[k];
    problem_->ExtractTrajectoryPoint(k, PROB::GetStateAtStep(xs, k),
                                     PROB::GetControlAtStep(us, k), &point);
  }

  if (problem_->enable_post_process()) {
    res.front().set_s(0.0);
    for (int i = 1; i < kHorizon; ++i) {
      const double d = (res[i].pos() - res[i - 1].pos()).norm();
      if (forward) {
        res[i].set_s(res[i - 1].s() + d);
      } else {
        res[i].set_s(res[i - 1].s() - d);
      }
    }
  }
  if (UNLIKELY(VLOG_IS_ON(3))) {
    for (int k = 0; k < kHorizon; ++k) {
      DDPVLOG(3) << "final_points[" << k << "]: " << res[k].DebugString();
    }
  }
  timer.Mark("result");

  // Warning.
  if (iteration > params_.max_iters()) {
    QLOG(WARNING) << owner_ << " ddp reach max iteration.";
    if (owner_ == "trajectory_optimizer") {
      QEVENT("runbing", "traj_opt_max_iteration",
             [&](QEvent *qevent) { qevent->AddField("iter", iteration); });
    } else if (owner_ == "freespace_local_smoother") {
      QEVENT("zhuang", "fs_local_smoother_max_iteration",
             [&](QEvent *qevent) { qevent->AddField("iter", iteration); });
    }
  }
  constexpr int kLineSearchQeventRecordLimit = 50;
  if (line_search_count >= kLineSearchQeventRecordLimit) {
    if (owner_ == "trajectory_optimizer") {
      QEVENT_EVERY_N_SECONDS("runbing", "traj_opt_line_search_count_large", 0.2,
                             [&](QEvent *qevent) {
                               qevent->AddField("line_search_count",
                                                line_search_count);
                             });
    }
  }
  return res;
}

template <typename PROB>
double DdpOptimizer<PROB>::EvaluateCost(
    const StatesType &xs, const ControlsType &us,
    absl::flat_hash_map<std::string, double> *cost_map) const {
  std::vector<double> costs(problem_->costs().size(), 0.0);

  for (int i = 0; i < problem_->costs().size(); ++i) {
    const auto divided_g = problem_->costs()[i]->SumGForAllSteps(xs, us);
    costs[i] += divided_g.sum();
    if (cost_map != nullptr) {
      for (const auto &[g_name, g] : divided_g.gs()) {
        (*cost_map)[g_name] = g;
      }
    }
  }
  const double total_cost = std::accumulate(costs.begin(), costs.end(), 0.0);
  if (UNLIKELY(VLOG_IS_ON(4))) {
    DDPVLOG(4) << "Total cost: " << total_cost;
    for (const auto &[cost_name, cost] : *cost_map) {
      DDPVLOG(4) << "  [" << cost_name << "] cost: " << cost;
    }
  }
  return total_cost;
}

template <typename PROB>
double DdpOptimizer<PROB>::LineSearchAndEvaluateCost(
    int iteration, const StatesType &tentative_xs,
    const ControlsType &tentative_us, const StatesType &full_dxs,
    const ControlsType &full_dus, double alpha,
    typename HookType::OptimizerInspector *oi) {
  SCOPED_QTRACE("LineSearchAndEvaluateCost");
  DDPVLOG(4) << "Evaluating cost for line search iteration " << iteration
             << " with alpha " << alpha;
  for (auto *hook : hooks_) {
    hook->OnLineSearchIterationStart(iteration, tentative_xs, tentative_us,
                                     full_dxs, full_dus, alpha, *oi);
  }
  const double cost = EvaluateCost(tentative_xs, tentative_us);
  oi->cost = cost;
  for (auto *hook : hooks_) {
    hook->OnLineSearchIterationEnd(iteration, tentative_xs, tentative_us,
                                   full_dxs, full_dus, alpha, cost, *oi);
  }
  return cost;
}

template <typename PROB>
double DdpOptimizer<PROB>::StepSizeAjustmentAndEvaluateCost(
    int iteration, const StatesType &xs, const ControlsType &us, int k_stepsize,
    typename HookType::OptimizerInspector *oi) {
  SCOPED_QTRACE("StepSizeAjustmentAndEvaluateCost");
  DDPVLOG(4) << "Evaluating cost for step-size adjustment iteration "
             << iteration << " with step " << k_stepsize;
  for (auto *hook : hooks_) {
    hook->OnStepSizeAdjustmentIterationStart(iteration, xs, us, k_stepsize,
                                             *oi);
  }
  const double cost = EvaluateCost(xs, us);
  oi->cost = cost;
  for (auto *hook : hooks_) {
    hook->OnStepSizeAdjustmentIterationEnd(iteration, xs, us, k_stepsize, cost,
                                           *oi);
  }
  return cost;
}

template <typename PROB>
void DdpOptimizer<PROB>::EvaluateEachDiscountedAccumulativeCost(
    const MfobDAT::StatesType &xs, const MfobDAT::ControlsType &us,
    double gamma, AccumulatedDiscountedCostsProto *costs) const {
  const std::string find_object_string = "Object";
  const std::string object_feature_name = "ObjectCost";
  const int soft_object_feature_idx = 0;
  const int hard_object_feature_idx = 1;
  const std::string object_soft_buffer_name = "Soft";
  const std::string object_hard_buffer_name = "Hard";
  std::vector<std::pair<std::string, double>> object_cost;
  object_cost.resize(/*object_cost_buffet_size=*/2, {"", 0.0});

  for (int i = 0; i < problem_->costs().size(); ++i) {
    double gamma_k = 1.0;
    std::vector<std::pair<std::string, double>> divide_cost;
    for (int k = 0; k < kDdpTrajectoryStepsDAT; ++k) {
      const auto cost_k = problem_->costs()[i]->EvaluateGWithDebugInfo(
          k, MfobDAT::GetStateAtStep(xs, k), MfobDAT::GetControlAtStep(us, k),
          /*using_scale=*/false);
      const auto &gs = cost_k.gs();
      const int gs_size = gs.size();
      if (divide_cost.size() < gs_size) {
        const int old_size = divide_cost.size();
        divide_cost.resize(gs_size);
        for (int idx = old_size; idx < gs_size; ++idx) {
          divide_cost[idx].first = gs[idx].first;
          divide_cost[idx].second = 0.0;
        }
      }
      for (int idx = 0; idx < gs_size; ++idx) {
        divide_cost[idx].second += gamma_k * gs[idx].second;
      }
      gamma_k *= gamma;
    }
    const auto &name = problem_->costs()[i]->name();
    if (name.find(find_object_string) != std::string::npos) {
      for (int idx = 0; idx < divide_cost.size(); ++idx) {
        const auto &name_cost = divide_cost[idx].first;
        if (name_cost.find(object_soft_buffer_name) != std::string::npos) {
          object_cost[soft_object_feature_idx].second +=
              divide_cost[idx].second;
        } else if (name_cost.find(object_hard_buffer_name) !=
                   std::string::npos) {
          object_cost[hard_object_feature_idx].second +=
              divide_cost[idx].second;
        }
      }
      continue;
    }
    for (int idx = 0; idx < divide_cost.size(); ++idx) {
      FeatureCostProto *feature_cost = costs->add_feature_costs();
      feature_cost->set_feature(divide_cost[idx].first);
      feature_cost->set_cost(divide_cost[idx].second);
    }
  }
  for (int idx = 0; idx < object_cost.size(); ++idx) {
    FeatureCostProto *object_feature_cost = costs->add_feature_costs();
    if (idx == soft_object_feature_idx) {
      object_feature_cost->set_feature(object_soft_buffer_name +
                                       object_feature_name);
    } else if (idx == hard_object_feature_idx) {
      object_feature_cost->set_feature(object_hard_buffer_name +
                                       object_feature_name);
    }
    object_feature_cost->set_cost(object_cost[idx].second);
  }

  if (UNLIKELY(VLOG_IS_ON(3))) {
    DDPVLOG(3) << "Discounted Accumulative Cost: ";
    for (int i = 0; i < costs->feature_costs_size(); ++i) {
      DDPVLOG(3) << "  [" << costs->feature_costs(i).feature()
                 << "] cost: " << costs->feature_costs(i).cost();
    }
  }
}

#undef DDPVLOG
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_
