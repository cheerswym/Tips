#ifndef ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_OPTIMIZER_
#define ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_OPTIMIZER_

#include <coin/IpDenseVector.hpp>
#include <coin/IpIpoptData.hpp>
#include <coin/IpIteratesVector.hpp>
#include <coin/IpSolveStatistics.hpp>
#include <coin/IpTNLP.hpp>
#include <limits>       // NOLINT
#include <string>       // NOLINT
#include <string_view>  // NOLINT
#include <utility>      // NOLINT
#include <vector>       // NOLINT

#include "onboard/lite/check.h"
#include "onboard/math/eigen.h"
#include "onboard/planner/optimization/ipopt/ipopt_optimizer_debug_hook.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/trajectory_point.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class IpoptOptimizer : public Ipopt::TNLP {
 public:
  static constexpr int kStateSize = PROB::kStateSize;
  static constexpr int kControlSize = PROB::kControlSize;
  static constexpr int kHorizon = PROB::kHorizon;

  static constexpr int kStatesSize = kStateSize * kHorizon;
  static constexpr int kControlsSize = kControlSize * kHorizon;
  static constexpr int kVariablesSize = kStatesSize + kControlsSize;

  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;

  using AlgorithmMode = Ipopt::AlgorithmMode;
  using IpoptData = Ipopt::IpoptData;
  using IpoptCalculatedQuantities = Ipopt::IpoptCalculatedQuantities;
  using SolverReturn = Ipopt::SolverReturn;

  using HookType = typename qcraft::planner::IpoptOptimizerDebugHook<PROB>;

  /**
   * @brief  Creates an IpoptOptimizer wrapping the nlp.
   *
   * This constructor holds and modifies the passed nlp.
   */
  IpoptOptimizer(const PROB* problem, std::string_view owner);
  virtual ~IpoptOptimizer() = default;

  void SetInitialPoints(std::vector<TrajectoryPoint> init_points);

  std::vector<TrajectoryPoint> GetFinalTraj() { return result_points_; }

  void AddHook(HookType* hook) { hooks_.push_back(hook); }

 private:
  /** Method to return some info about the nlp */
  bool get_nlp_info(int& n, int& m, int& nnz_jac_g,         // NOLINT
                    int& nnz_h_lag,                         // NOLINT
                    IndexStyleEnum& index_style) override;  // NOLINT

  /** Method to return the bounds for my problem */
  bool get_bounds_info(int n, double* x_l, double* x_u, int m, double* g_l,
                       double* g_u) override;

  /** Method to return the starting point for the algorithm */
  bool get_starting_point(int n, bool init_x, double* x, bool init_z,
                          double* z_L, double* z_U, int m, bool init_lambda,
                          double* lambda) override;

  /** Method to return the objective value */
  bool eval_f(int n, const double* x, bool new_x,
              double& obj_value) override;  // NOLINT

  /** Method to return the gradient of the objective */
  bool eval_grad_f(int n, const double* x, bool new_x, double* grad_f) override;

  /** Method to return the constraint residuals */
  bool eval_g(int n, const double* x, bool new_x, int m, double* g) override;

  /** Method to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  bool eval_jac_g(int n, const double* x, bool new_x, int m, int nele_jac,
                  int* iRow, int* jCol, double* values) override;

  /** This is called after every iteration and is used to save intermediate
   *  solutions in the nlp */
  bool intermediate_callback(AlgorithmMode mode, int iter, double obj_value,
                             double inf_pr, double inf_du, double mu,
                             double d_norm, double regularization_size,
                             double alpha_du, double alpha_pr, int ls_trials,
                             const IpoptData* ip_data,
                             IpoptCalculatedQuantities* ip_cq) override;

  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  void finalize_solution(SolverReturn status, int n, const double* x,
                         const double* z_L, const double* z_U, int m,
                         const double* g, const double* lambda,
                         double obj_value, const IpoptData* ip_data,
                         IpoptCalculatedQuantities* ip_cq) override;

  bool UpdateCosts(const StatesType& xs, const ControlsType& us,
                   bool value_only) {
    // Check if xs and us have NaN of Inf, if so return false to stop solving.
    constexpr double kMaxValue = 1e50;
    for (int k = 0; k < kStatesSize; ++k) {
      if (xs[k] > kMaxValue || xs[k] < -kMaxValue || std::isnan(xs[k])) {
        VLOG(2) << "Intermediate_callback check xs inf or nan: " << k << " "
                << xs[k];
        return false;
      }
    }
    for (int k = 0; k < kControlsSize; ++k) {
      if (us[k] > kMaxValue || us[k] < -kMaxValue || std::isnan(us[k])) {
        VLOG(2) << "Intermediate_callback check us inf or nan: " << k << " "
                << us[k];
        return false;
      }
    }
    for (const auto& cost : problem_->costs()) {
      cost.get()->Update(xs, us, value_only);
    }
    return true;
  }

  double EvaluateCost(
      const StatesType& xs, const ControlsType& us,
      absl::flat_hash_map<std::string, double>* cost_map) const {
    std::vector<double> costs(problem_->costs().size(), 0.0);
    for (int i = 0; i < problem_->costs().size(); ++i) {
      costs[i] += problem_->costs()[i]->SumGForAllSteps(xs, us).sum();
    }
    const double total_cost = std::accumulate(costs.begin(), costs.end(), 0.0);
    if (cost_map != nullptr) {
      cost_map->clear();
      for (int i = 0; i < problem_->costs().size(); ++i) {
        (*cost_map)[problem_->costs()[i]->name()] = costs[i];
      }
    }
    return total_cost;
  }

  const PROB* problem_;
  std::string owner_;

  std::vector<TrajectoryPoint> init_points_;
  std::vector<TrajectoryPoint> result_points_;

  std::vector<HookType*> hooks_;
};

template <typename PROB>
IpoptOptimizer<PROB>::IpoptOptimizer(const PROB* problem,
                                     std::string_view owner)
    : problem_(CHECK_NOTNULL(problem)), owner_(owner) {}

template <typename PROB>
void IpoptOptimizer<PROB>::SetInitialPoints(
    std::vector<TrajectoryPoint> init_points) {
  QCHECK_EQ(init_points.size(), kHorizon);
  init_points_ = std::move(init_points);
  init_points_.front().set_theta(
      init_points_[1].theta() +
      NormalizeAngle(init_points_.front().theta() - init_points_[1].theta()));
  for (int k = 0; k < kHorizon; ++k) init_points_[k].set_t(k * problem_->dt());
}

template <typename PROB>
bool IpoptOptimizer<PROB>::get_nlp_info(
    int& n, int& m, int& nnz_jac_g,  // NOLINT
    int& nnz_h_lag,                  // NOLINT
    IndexStyleEnum& index_style) {   // NOLINT
  n = kVariablesSize;                // states + controls
  m = (kHorizon - 1) * kStateSize;   // (x0 + kinematic) * kHorizon

  nnz_jac_g = m * (kStateSize + 1 + kControlSize);

  nnz_h_lag = n * (n + 1) / 2;

  // start index at 0 for row/col entries
  index_style = C_STYLE;

  return true;
}

template <typename PROB>
bool IpoptOptimizer<PROB>::get_bounds_info(int n, double* x_l, double* x_u,
                                           int m, double* g_l, double* g_u) {
  const StateType x0 = problem_->FitInitialState(init_points_);
  for (std::size_t c = 0; c < n; ++c) {
    if (c < kStateSize) {
      x_l[c] = x0[c];
      x_u[c] = x0[c];
    } else {
      x_l[c] = -std::numeric_limits<double>::infinity();
      x_u[c] = std::numeric_limits<double>::infinity();
    }
  }

  // Specific bounds depending on equality and inequality constraints
  for (std::size_t c = 0; c < m; ++c) {
    g_l[c] = 0.0;
    g_u[c] = 0.0;
  }

  return true;
}

template <typename PROB>
bool IpoptOptimizer<PROB>::get_starting_point(int n, bool init_x, double* x,
                                              bool init_z, double* z_L,
                                              double* z_U, int m,
                                              bool init_lambda,
                                              double* lambda) {
  VLOG(4) << "---------------get_starting_point------------------";

  // Here, we assume we only have starting values for x
  QCHECK(init_x);
  QCHECK(!init_z);
  QCHECK(!init_lambda);

  QCHECK_EQ(init_points_.size(), kHorizon);
  const StateType x0 = problem_->FitInitialState(init_points_);
  const ControlsType init_us = problem_->FitControl(init_points_, x0);
  const StatesType init_xs = problem_->FitState(init_points_);

  Eigen::Map<StatesType>(&x[0], kStatesSize) = init_xs;
  Eigen::Map<ControlsType>(&x[kStatesSize], kControlsSize) = init_us;

  // Add solve start debug information here
  if (!UpdateCosts(init_xs, init_us, /*value_only=*/true)) return false;
  typename HookType::OptimizerInspector oi;
  {
    const double init_cost = EvaluateCost(init_xs, init_us, &oi.cost_map);
    VLOG(1) << "Init cost: " << init_cost;
    if (VLOG_IS_ON(2)) {
      for (int i = 0; i < problem_->costs().size(); ++i) {
        VLOG(2) << "  [" << problem_->costs()[i]->name()
                << "] cost: " << oi.cost_map[problem_->costs()[i]->name()];
      }
    }
    oi.cost = init_cost;
    for (const auto& hook : hooks_) {
      hook->OnSolveStart(init_xs, init_us, oi);
    }
  }

  return true;
}

template <typename PROB>
bool IpoptOptimizer<PROB>::eval_f(int n, const double* x, bool new_x,
                                  double& obj_value) {  // NOLINT
  VLOG(4) << "---------------eval_f---------------";
  const auto xs = Eigen::Map<const StatesType>(&x[0], kStatesSize);
  const auto us =
      Eigen::Map<const ControlsType>(&x[kStatesSize], kControlsSize);
  if (!UpdateCosts(xs, us, /*value_only=*/true)) return false;
  obj_value = EvaluateCost(xs, us, nullptr);
  return true;
}

template <typename PROB>
bool IpoptOptimizer<PROB>::eval_grad_f(int n, const double* x, bool new_x,
                                       double* grad_f) {
  VLOG(4) << "---------------eval_grad_f---------------";
  const auto xs = Eigen::Map<const StatesType>(&x[0], kStatesSize);
  const auto us =
      Eigen::Map<const ControlsType>(&x[kStatesSize], kControlsSize);
  if (!UpdateCosts(xs, us, /*value_only=*/true)) return false;
  for (int k = 0; k < kHorizon; ++k) {
    DGDxType dgdx = DGDxType::Zero();
    DGDuType dgdu = DGDuType::Zero();

    const auto x_k = PROB::GetStateAtStep(xs, k);
    const auto u_k = PROB::GetControlAtStep(us, k);
    for (const auto& cost : problem_->costs()) {
      dgdx += cost->EvaluateDGDx(k, x_k, u_k);
      dgdu += cost->EvaluateDGDu(k, x_k, u_k);
    }
    Eigen::Map<StateType>(&grad_f[kStateSize * k], kStateSize) =
        dgdx.transpose();
    Eigen::Map<ControlType>(&grad_f[kStatesSize + k * kControlSize],
                            kControlSize) = dgdu.transpose();
  }
  return true;
}

template <typename PROB>
bool IpoptOptimizer<PROB>::eval_g(int n, const double* x, bool new_x, int m,
                                  double* g) {
  VLOG(4) << "---------------eval_g---------------";
  const auto xs = Eigen::Map<const StatesType>(&x[0], kStatesSize);
  const auto us =
      Eigen::Map<const ControlsType>(&x[kStatesSize], kControlsSize);
  for (int k = 0; k < kHorizon - 1; ++k) {
    const auto x_k = PROB::GetStateAtStep(xs, k);
    const auto u_k = PROB::GetControlAtStep(us, k);
    const auto state_next = problem_->EvaluateF(k, x_k, u_k);
    Eigen::Map<StateType>(&g[kStateSize * k], kStateSize) =
        state_next - PROB::GetStateAtStep(xs, k + 1);
  }
  return true;
}

// sparse
template <typename PROB>
bool IpoptOptimizer<PROB>::eval_jac_g(int n, const double* x, bool new_x, int m,
                                      int nele_jac, int* iRow, int* jCol,
                                      double* values) {
  VLOG(4) << "---------------eval_jac_g---------------";
  if (values == NULL) {
    // Init jacobi location
    int nele = 0;
    for (int k = 0; k < kHorizon - 1; ++k) {
      for (int i = 0; i < kStateSize; ++i) {
        for (int j = 0; j < kStateSize; ++j) {
          iRow[nele] = k * kStateSize + i;  // constraint
          jCol[nele] = k * kStateSize + j;  // variable
          ++nele;
        }
        iRow[nele] = k * kStateSize + i;               // constraint
        jCol[nele] = k * kStateSize + kStateSize + i;  // variable
        ++nele;
        for (int j = 0; j < kControlSize; ++j) {
          iRow[nele] = k * kStateSize + i;                  // constraint
          jCol[nele] = kStatesSize + k * kControlSize + j;  // variable
          ++nele;
        }
      }
    }
  } else {
    const auto xs = Eigen::Map<const StatesType>(&x[0], kStatesSize);
    const auto us =
        Eigen::Map<const ControlsType>(&x[kStatesSize], kControlsSize);
    int nele = 0;
    for (int k = 0; k < kHorizon - 1; ++k) {
      typename PROB::FDerivatives fd;
      problem_->EvaluateFDerivatives(k, PROB::GetStateAtStep(xs, k),
                                     PROB::GetControlAtStep(us, k), &fd);
      for (int i = 0; i < kStateSize; ++i) {
        for (int j = 0; j < kStateSize; ++j) {
          values[nele] = fd.dfdx(i, j);
          ++nele;
        }
        values[nele] = -1.0;
        ++nele;
        for (int j = 0; j < kControlSize; ++j) {
          values[nele] = fd.dfdu(i, j);
          ++nele;
        }
      }
    }
  }
  return true;
}

template <typename PROB>
bool IpoptOptimizer<PROB>::intermediate_callback(
    AlgorithmMode mode, int iter, double obj_value, double inf_pr,
    double inf_du, double mu, double d_norm, double regularization_size,
    double alpha_du, double alpha_pr, int ls_trials, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq) {
  VLOG(4) << "---------------intermediate_callback---------------";
  // Add iteration debug information here
  // Ip_date->curr()->x() without x0 state.
  const double* x =
      static_cast<const Ipopt::DenseVector*>(&(*ip_data->curr()->x()))
          ->Values();
  const auto xs_no_start =
      Eigen::Map<const Eigen::Matrix<double, kStateSize*(kHorizon - 1), 1>>(
          &x[0], kStatesSize - kStateSize);
  const auto us = Eigen::Map<const ControlsType>(&x[kStatesSize - kStateSize],
                                                 kControlsSize);
  StatesType xs;
  xs.template segment<kStateSize>(0) = problem_->FitInitialState(init_points_);
  xs.template segment<kStatesSize - kStateSize>(kStateSize) = xs_no_start;

  if (!UpdateCosts(xs, us, /*value_only=*/true)) return false;

  typename HookType::OptimizerInspector oi;
  {
    const double cost = EvaluateCost(xs, us, &oi.cost_map);
    VLOG(1) << "Iteration: " << iter << " cost: " << cost;
    if (VLOG_IS_ON(2)) {
      for (int i = 0; i < problem_->costs().size(); ++i) {
        VLOG(2) << "  [" << problem_->costs()[i]->name()
                << "] cost: " << oi.cost_map[problem_->costs()[i]->name()];
      }
    }
    oi.cost = cost;
    for (const auto& hook : hooks_) {
      hook->OnIterationEnd(iter, xs, us, oi);
    }
  }
  return true;
}

template <typename PROB>
void IpoptOptimizer<PROB>::finalize_solution(
    SolverReturn status, int n, const double* x, const double* z_L,
    const double* z_U, int m, const double* g, const double* lambda,
    double obj_value, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq) {
  VLOG(4) << "---------------finalize_solution---------------";
  const auto xs = Eigen::Map<const StatesType>(&x[0], kStatesSize);
  const auto us =
      Eigen::Map<const ControlsType>(&x[kStatesSize], kControlsSize);

  // Add final debug information here
  UpdateCosts(xs, us, /*value_only=*/true);
  typename HookType::OptimizerInspector oi;
  {
    const double cost = EvaluateCost(xs, us, &oi.cost_map);
    VLOG(1) << "Final: " << cost;
    if (VLOG_IS_ON(2)) {
      for (int i = 0; i < problem_->costs().size(); ++i) {
        VLOG(2) << "  [" << problem_->costs()[i]->name()
                << "] cost: " << oi.cost_map[problem_->costs()[i]->name()];
      }
    }
    oi.cost = cost;
    for (const auto& hook : hooks_) {
      hook->OnSolveEnd(xs, us, oi);
    }
  }

  result_points_.resize(kHorizon);
  for (int k = 0; k < kHorizon; ++k) {
    TrajectoryPoint& point = result_points_[k];
    problem_->ExtractTrajectoryPoint(k, PROB::GetStateAtStep(xs, k),
                                     PROB::GetControlAtStep(us, k), &point);
  }
  result_points_.front().set_s(0.0);
  for (int i = 1; i < kHorizon; ++i) {
    const double d =
        (result_points_[i].pos() - result_points_[i - 1].pos()).norm();
    result_points_[i].set_s(result_points_[i - 1].s() + d);
  }
}

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_IPOPT_IPOPT_OPTIMIZER_
