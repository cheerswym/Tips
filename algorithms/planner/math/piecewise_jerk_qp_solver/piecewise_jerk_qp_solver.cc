#include "onboard/planner/math/piecewise_jerk_qp_solver/piecewise_jerk_qp_solver.h"

#include <cmath>
#include <limits>

#include "glog/logging.h"
#include "onboard/eval/qevent.h"
#include "onboard/lite/logging.h"
#include "onboard/math/qp/osqp_return_status.h"
#include "onboard/planner/planner_flags.h"

DEFINE_bool(planner_enable_piecewise_jerk_qp_solver_debug, false,
            "Enable osqp debug mode.");

DEFINE_bool(planner_use_osqp_max_iteration_result, true,
            "Use the result when osqp return OSQP_MAX_ITER_REACHED status.");

namespace qcraft {

namespace {
// The maximum distance of first order variable
constexpr double kMaxVariableRange = 200.0;
}  // namespace

PiecewiseJerkQpSolver::PiecewiseJerkQpSolver(
    int num_knots, double delta_t, int x_lower_soft_constraint_type_num,
    int x_upper_soft_constraint_type_num, int dx_lower_soft_constraint_type_num,
    int dx_upper_soft_constraint_type_num,
    int ddx_lower_soft_constraint_type_num,
    int ddx_upper_soft_constraint_type_num)
    : num_knots_(num_knots),
      x_lower_soft_constraint_type_num_(x_lower_soft_constraint_type_num),
      x_upper_soft_constraint_type_num_(x_upper_soft_constraint_type_num),
      dx_lower_soft_constraint_type_num_(dx_lower_soft_constraint_type_num),
      dx_upper_soft_constraint_type_num_(dx_upper_soft_constraint_type_num),
      ddx_lower_soft_constraint_type_num_(ddx_lower_soft_constraint_type_num),
      ddx_upper_soft_constraint_type_num_(ddx_upper_soft_constraint_type_num),
      delta_t_(delta_t),
      delta_t_sq_(delta_t_ * delta_t_),
      delta_t_tri_(delta_t_sq_ * delta_t_) {
  QCHECK_GE(num_knots_, 3);

  total_num_of_var_ =
      num_knots_ * 4 + x_lower_soft_constraint_type_num_ +
      x_upper_soft_constraint_type_num_ + dx_lower_soft_constraint_type_num_ +
      dx_upper_soft_constraint_type_num_ + ddx_lower_soft_constraint_type_num_ +
      ddx_upper_soft_constraint_type_num_;

  weight_.x_w.resize(num_knots_, 0.0);
  weight_.dx_w.resize(num_knots_, 0.0);
  weight_.ddx_w.resize(num_knots_, 0.0);
  weight_.dddx_w.resize(num_knots_, 0.0);
  weight_.x_lower_slack_w.resize(x_lower_soft_constraint_type_num_, 0.0);
  weight_.x_upper_slack_w.resize(x_upper_soft_constraint_type_num_, 0.0);
  weight_.dx_lower_slack_w.resize(dx_lower_soft_constraint_type_num_, 0.0);
  weight_.dx_upper_slack_w.resize(dx_upper_soft_constraint_type_num_, 0.0);
  weight_.ddx_lower_slack_w.resize(ddx_lower_soft_constraint_type_num_, 0.0);
  weight_.ddx_upper_slack_w.resize(ddx_upper_soft_constraint_type_num_, 0.0);
  offset_.resize(total_num_of_var_, 0.0);

  x_lower_slack_start_idx = 4 * num_knots_;
  x_upper_slack_start_idx =
      x_lower_slack_start_idx + x_lower_soft_constraint_type_num_;
  dx_lower_slack_start_idx =
      x_upper_slack_start_idx + x_upper_soft_constraint_type_num_;
  dx_upper_slack_start_idx =
      dx_lower_slack_start_idx + dx_lower_soft_constraint_type_num_;
  ddx_lower_slack_start_idx =
      dx_upper_slack_start_idx + dx_upper_soft_constraint_type_num_;
  ddx_upper_slack_start_idx =
      ddx_lower_slack_start_idx + ddx_lower_soft_constraint_type_num_;

  constraints_data_cols_.resize(total_num_of_var_);
  constraints_indice_cols_.resize(total_num_of_var_);
}

PiecewiseJerkQpSolver::PiecewiseJerkQpSolver(int num_knots, double delta_t)
    : PiecewiseJerkQpSolver(num_knots, delta_t, 0, 0, 0, 0, 0, 0) {}

void PiecewiseJerkQpSolver::ReInit() {
  segments_.clear();

  constraints_data_cols_.clear();
  constraints_data_cols_.resize(total_num_of_var_);

  constraints_indice_cols_.clear();
  constraints_indice_cols_.resize(total_num_of_var_);

  lower_bounds_.clear();
  upper_bounds_.clear();

  offset_ = std::vector<c_float>(total_num_of_var_, 0.0);

  weight_.x_w = std::vector<double>(num_knots_, 0.0);
  weight_.dx_w = std::vector<double>(num_knots_, 0.0);
  weight_.ddx_w = std::vector<double>(num_knots_, 0.0);
  weight_.dddx_w = std::vector<double>(num_knots_, 0.0);
  weight_.x_lower_slack_w =
      std::vector<double>(x_lower_soft_constraint_type_num_, 0.0);
  weight_.x_upper_slack_w =
      std::vector<double>(x_upper_soft_constraint_type_num_, 0.0);
  weight_.dx_lower_slack_w =
      std::vector<double>(dx_lower_soft_constraint_type_num_, 0.0);
  weight_.dx_upper_slack_w =
      std::vector<double>(dx_upper_soft_constraint_type_num_, 0.0);
  weight_.ddx_lower_slack_w =
      std::vector<double>(ddx_lower_soft_constraint_type_num_, 0.0);
  weight_.ddx_upper_slack_w =
      std::vector<double>(ddx_upper_soft_constraint_type_num_, 0.0);

  curr_num_constraints_ = 0;
}

void PiecewiseJerkQpSolver::AddNthOrderReferencePointKernel(int order,
                                                            int index,
                                                            double ref_val,
                                                            double weight) {
  QCHECK_LE(order, 3);
  QCHECK_GE(order, 0);
  QCHECK_LT(index, num_knots_);

  if (order == 0) {
    weight_.x_w[index] += 2.0 * weight;
  } else if (order == 1) {
    weight_.dx_w[index] += 2.0 * weight;
  } else if (order == 2) {
    weight_.ddx_w[index] += 2.0 * weight;
  } else {
    weight_.dddx_w[index] += 2.0 * weight;
  }

  offset_[index + num_knots_ * order] -= 2.0 * weight * ref_val;
}

void PiecewiseJerkQpSolver::AddNthOrderWeight(int order, int index,
                                              double weight) {
  QCHECK_LE(order, 3);
  QCHECK_GE(order, 0);
  QCHECK_LT(index, num_knots_);

  if (order == 0) {
    weight_.x_w[index] += 2.0 * weight;
  } else if (order == 1) {
    weight_.dx_w[index] += 2.0 * weight;
  } else if (order == 2) {
    weight_.ddx_w[index] += 2.0 * weight;
  } else {
    weight_.dddx_w[index] += 2.0 * weight;
  }
}

void PiecewiseJerkQpSolver::AddFirstOrderDerivativeKernel(double weight) {
  for (int i = 0; i < weight_.dx_w.size(); ++i) {
    weight_.dx_w[i] += 2.0 * weight;
  }
}

void PiecewiseJerkQpSolver::AddSecondOrderDerivativeKernel(double weight) {
  for (int i = 0; i < weight_.dx_w.size(); ++i) {
    weight_.ddx_w[i] += 2.0 * weight;
  }
}

void PiecewiseJerkQpSolver::AddThirdOrderDerivativeKernel(double weight) {
  for (int i = 0; i < weight_.dx_w.size(); ++i) {
    weight_.dddx_w[i] += 2.0 * weight;
  }
}

void PiecewiseJerkQpSolver::AddZeroOrderSlackVarKernel(
    BoundType bound_type, absl::Span<const int> slack_indices,
    absl::Span<const double> weights) {
  QCHECK_EQ(slack_indices.size(), weights.size());
  for (int i = 0; i < slack_indices.size(); ++i) {
    const int index = slack_indices[i];
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(index, x_lower_soft_constraint_type_num_);
      weight_.x_lower_slack_w[index] += 2.0 * weights[i];
    } else {
      QCHECK_LT(index, x_upper_soft_constraint_type_num_);
      weight_.x_upper_slack_w[index] += 2.0 * weights[i];
    }
  }
}

void PiecewiseJerkQpSolver::AddFirstOrderSlackVarKernel(
    BoundType bound_type, absl::Span<const int> slack_indices,
    absl::Span<const double> weights) {
  QCHECK_EQ(slack_indices.size(), weights.size());
  for (int i = 0; i < slack_indices.size(); ++i) {
    const int index = slack_indices[i];
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(index, dx_lower_soft_constraint_type_num_);
      weight_.dx_lower_slack_w[index] += 2.0 * weights[i];
    } else {
      QCHECK_LT(index, dx_upper_soft_constraint_type_num_);
      weight_.dx_upper_slack_w[index] += 2.0 * weights[i];
    }
  }
}

void PiecewiseJerkQpSolver::AddSecondOrderSlackVarKernel(
    BoundType bound_type, absl::Span<const int> slack_indices,
    absl::Span<const double> weights) {
  QCHECK_EQ(slack_indices.size(), weights.size());
  for (int i = 0; i < slack_indices.size(); ++i) {
    const int index = slack_indices[i];
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(index, ddx_lower_soft_constraint_type_num_);
      weight_.ddx_lower_slack_w[index] += 2.0 * weights[i];
    } else {
      QCHECK_LT(index, ddx_upper_soft_constraint_type_num_);
      weight_.ddx_upper_slack_w[index] += 2.0 * weights[i];
    }
  }
}

void PiecewiseJerkQpSolver::AddRegularization(double delta) {
  for (int i = 0; i < num_knots_; ++i) {
    weight_.x_w[i] += delta;
    weight_.dx_w[i] += delta;
    weight_.ddx_w[i] += delta;
    weight_.dddx_w[i] += delta;
  }
}

void PiecewiseJerkQpSolver::AddNthOrderEqualityConstraint(
    int order, absl::Span<const int> index_list, absl::Span<const double> coeff,
    double value) {
  QCHECK_LE(order, 3);
  AddNthOrderInequalityConstraint(order, index_list, coeff, value, value);
}

void PiecewiseJerkQpSolver::AddNthOrderInequalityConstraint(
    int order, absl::Span<const int> index_list, absl::Span<const double> coeff,
    double lower_bound, double upper_bound) {
  QCHECK_LE(order, 3);
  QCHECK_EQ(index_list.size(), coeff.size());
  QCHECK_LE(lower_bound, upper_bound)
      << "order = " << order << "lower_bound = " << lower_bound
      << ", upper_bound = " << upper_bound
      << ", index_list.size() = " << index_list.size()
      << ", index_list[0] = " << index_list.front();

  for (int i = 0; i < index_list.size(); ++i) {
    const int idx = index_list[i] + order * num_knots_;

    QCHECK_LT(idx, 4 * num_knots_);
    QCHECK_GE(idx, order * num_knots_);
    QCHECK_LT(idx, (order + 1) * num_knots_);

    constraints_data_cols_[idx].push_back(coeff[i]);
    constraints_indice_cols_[idx].push_back(curr_num_constraints_);
  }
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);
  ++curr_num_constraints_;
  QCHECK_EQ(lower_bounds_.size(), curr_num_constraints_);
  QCHECK_EQ(upper_bounds_.size(), curr_num_constraints_);
}

void PiecewiseJerkQpSolver::AddZeroOrderInequalityConstraintWithOneSideSlack(
    absl::Span<const int> index_list, absl::Span<const double> coeff,
    int slack_term_index, BoundType bound_type, double lower_bound,
    double upper_bound) {
  QCHECK_EQ(index_list.size(), coeff.size());
  QCHECK_LE(lower_bound, upper_bound);

  const int global_slack_idx = bound_type == BoundType::LOWER_BOUND
                                   ? x_lower_slack_start_idx + slack_term_index
                                   : x_upper_slack_start_idx + slack_term_index;
  for (int i = 0; i < index_list.size(); ++i) {
    const int idx = index_list[i];
    QCHECK_LT(idx, num_knots_);
    constraints_data_cols_[idx].push_back(1.0);
    constraints_indice_cols_[idx].push_back(curr_num_constraints_);
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(slack_term_index, x_lower_soft_constraint_type_num_);
      constraints_data_cols_[global_slack_idx].push_back(1.0);
      constraints_indice_cols_[global_slack_idx].push_back(
          curr_num_constraints_);
    } else if (bound_type == BoundType::UPPER_BOUND) {
      QCHECK_LT(slack_term_index, x_upper_soft_constraint_type_num_);
      constraints_data_cols_[global_slack_idx].push_back(-1.0);
      constraints_indice_cols_[global_slack_idx].push_back(
          curr_num_constraints_);
    }
  }
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);
  ++curr_num_constraints_;

  // Add slack variable constraint.
  constraints_data_cols_[global_slack_idx].push_back(1.0);
  constraints_indice_cols_[global_slack_idx].push_back(curr_num_constraints_++);
  lower_bounds_.push_back(0.0);
  upper_bounds_.push_back(std::numeric_limits<double>::infinity());

  QCHECK_EQ(lower_bounds_.size(), curr_num_constraints_);
  QCHECK_EQ(upper_bounds_.size(), curr_num_constraints_);
}

void PiecewiseJerkQpSolver::AddFirstOrderInequalityConstraintWithOneSideSlack(
    absl::Span<const int> index_list, absl::Span<const double> coeff,
    int slack_term_index, BoundType bound_type, double lower_bound,
    double upper_bound) {
  QCHECK_EQ(index_list.size(), coeff.size());
  QCHECK_LE(lower_bound, upper_bound);

  const int global_slack_idx =
      bound_type == BoundType::LOWER_BOUND
          ? dx_lower_slack_start_idx + slack_term_index
          : dx_upper_slack_start_idx + slack_term_index;
  for (int i = 0; i < index_list.size(); ++i) {
    const int idx = index_list[i] + num_knots_;
    QCHECK_LT(idx, 2 * num_knots_);
    // Add first order variable constraint.
    constraints_data_cols_[idx].push_back(1.0);
    constraints_indice_cols_[idx].push_back(curr_num_constraints_);
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(slack_term_index, dx_lower_soft_constraint_type_num_);
      constraints_data_cols_[global_slack_idx].push_back(1.0);
      constraints_indice_cols_[global_slack_idx].push_back(
          curr_num_constraints_);
    } else if (bound_type == BoundType::UPPER_BOUND) {
      QCHECK_LT(slack_term_index, dx_upper_soft_constraint_type_num_);
      constraints_data_cols_[global_slack_idx].push_back(-1.0);
      constraints_indice_cols_[global_slack_idx].push_back(
          curr_num_constraints_);
    }
  }
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);
  ++curr_num_constraints_;

  // Add slack variable constraint.
  constraints_data_cols_[global_slack_idx].push_back(1.0);
  constraints_indice_cols_[global_slack_idx].push_back(curr_num_constraints_++);
  lower_bounds_.push_back(0.0);
  upper_bounds_.push_back(std::numeric_limits<double>::infinity());

  QCHECK_EQ(lower_bounds_.size(), curr_num_constraints_);
  QCHECK_EQ(upper_bounds_.size(), curr_num_constraints_);
}

void PiecewiseJerkQpSolver::AddSecondOrderInequalityConstraintWithOneSideSlack(
    absl::Span<const int> index_list, absl::Span<const double> coeff,
    int slack_term_index, BoundType bound_type, double lower_bound,
    double upper_bound) {
  QCHECK_EQ(index_list.size(), coeff.size());
  QCHECK_LE(lower_bound, upper_bound);

  const int global_slack_idx =
      bound_type == BoundType::LOWER_BOUND
          ? ddx_lower_slack_start_idx + slack_term_index
          : ddx_upper_slack_start_idx + slack_term_index;
  for (int i = 0; i < index_list.size(); ++i) {
    const int idx = index_list[i] + 2 * num_knots_;
    QCHECK_LT(idx, 3 * num_knots_);
    constraints_data_cols_[idx].push_back(1.0);
    constraints_indice_cols_[idx].push_back(curr_num_constraints_);
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(slack_term_index, ddx_lower_soft_constraint_type_num_);
      constraints_data_cols_[global_slack_idx].push_back(1.0);
      constraints_indice_cols_[global_slack_idx].push_back(
          curr_num_constraints_);
    } else if (bound_type == BoundType::UPPER_BOUND) {
      QCHECK_LT(slack_term_index, ddx_upper_soft_constraint_type_num_);
      constraints_data_cols_[global_slack_idx].push_back(-1.0);
      constraints_indice_cols_[global_slack_idx].push_back(
          curr_num_constraints_);
    }
  }
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);
  ++curr_num_constraints_;

  // Add slack variable constraint.
  constraints_data_cols_[global_slack_idx].push_back(1.0);
  constraints_indice_cols_[global_slack_idx].push_back(curr_num_constraints_++);
  lower_bounds_.push_back(0.0);
  upper_bounds_.push_back(std::numeric_limits<double>::infinity());

  QCHECK_EQ(lower_bounds_.size(), curr_num_constraints_);
  QCHECK_EQ(upper_bounds_.size(), curr_num_constraints_);
}

void PiecewiseJerkQpSolver::AddInequalityConstraint(
    absl::Span<const int> index_list, absl::Span<const double> coeff,
    double lower_bound, double upper_bound) {
  QCHECK_EQ(index_list.size(), coeff.size());
  QCHECK_LE(lower_bound, upper_bound)
      << "lower_bound = " << lower_bound << ", upper_bound = " << upper_bound;

  for (int i = 0; i < index_list.size(); ++i) {
    const int idx = index_list[i];

    QCHECK_LT(idx, 4 * num_knots_);

    constraints_data_cols_[idx].push_back(coeff[i]);
    constraints_indice_cols_[idx].push_back(curr_num_constraints_);
  }
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);
  ++curr_num_constraints_;

  QCHECK_EQ(lower_bounds_.size(), curr_num_constraints_);
  QCHECK_EQ(upper_bounds_.size(), curr_num_constraints_);
}

void PiecewiseJerkQpSolver::AddIntrinsicConstraint() {
  for (int i = 0; i + 1 < num_knots_; ++i) {
    constraints_data_cols_[i].push_back(1.0);
    constraints_data_cols_[i + 1].push_back(-1.0);
    constraints_data_cols_[i + num_knots_].push_back(delta_t_);
    constraints_data_cols_[i + 2 * num_knots_].push_back(0.5 * delta_t_sq_);
    constraints_data_cols_[i + 3 * num_knots_].push_back(0.166667 *
                                                         delta_t_tri_);

    constraints_indice_cols_[i].push_back(curr_num_constraints_);
    constraints_indice_cols_[i + 1].push_back(curr_num_constraints_);
    constraints_indice_cols_[i + num_knots_].push_back(curr_num_constraints_);
    constraints_indice_cols_[i + 2 * num_knots_].push_back(
        curr_num_constraints_);
    constraints_indice_cols_[i + 3 * num_knots_].push_back(
        curr_num_constraints_);

    lower_bounds_.push_back(0.0);
    upper_bounds_.push_back(0.0);

    curr_num_constraints_++;

    constraints_data_cols_[i + num_knots_].push_back(1.0);
    constraints_data_cols_[i + num_knots_ + 1].push_back(-1.0);
    constraints_data_cols_[i + 2 * num_knots_].push_back(delta_t_);
    constraints_data_cols_[i + 3 * num_knots_].push_back(0.5 * delta_t_sq_);

    constraints_indice_cols_[i + num_knots_].push_back(curr_num_constraints_);
    constraints_indice_cols_[i + num_knots_ + 1].push_back(
        curr_num_constraints_);
    constraints_indice_cols_[i + 2 * num_knots_].push_back(
        curr_num_constraints_);
    constraints_indice_cols_[i + 3 * num_knots_].push_back(
        curr_num_constraints_);

    lower_bounds_.push_back(0.0);
    upper_bounds_.push_back(0.0);

    curr_num_constraints_++;

    constraints_data_cols_[i + 2 * num_knots_].push_back(1.0);
    constraints_data_cols_[i + 2 * num_knots_ + 1].push_back(-1.0);
    constraints_data_cols_[i + 3 * num_knots_].push_back(delta_t_);

    constraints_indice_cols_[i + 2 * num_knots_].push_back(
        curr_num_constraints_);
    constraints_indice_cols_[i + 2 * num_knots_ + 1].push_back(
        curr_num_constraints_);
    constraints_indice_cols_[i + 3 * num_knots_].push_back(
        curr_num_constraints_);

    lower_bounds_.push_back(0.0);
    upper_bounds_.push_back(0.0);

    curr_num_constraints_++;
  }
}

void PiecewiseJerkQpSolver::AddMonotonicConstraints() {
  for (int i = 0; i + 1 < num_knots_; ++i) {
    constraints_data_cols_[i].push_back(-1.0);
    constraints_data_cols_[i + 1].push_back(1.0);

    constraints_indice_cols_[i].push_back(curr_num_constraints_);
    constraints_indice_cols_[i + 1].push_back(curr_num_constraints_);

    lower_bounds_.push_back(0.0f);
    upper_bounds_.push_back(kMaxVariableRange);
    curr_num_constraints_++;
  }
}

void PiecewiseJerkQpSolver::SetInitConditions(
    const std::array<double, 3>& x_init) {
  x_init_ = x_init;
  AddNthOrderEqualityConstraint(0, std::vector<int>{0},
                                std::vector<double>{1.0}, x_init[0]);
  AddNthOrderEqualityConstraint(1, std::vector<int>{0},
                                std::vector<double>{1.0}, x_init[1]);
  AddNthOrderEqualityConstraint(2, std::vector<int>{0},
                                std::vector<double>{1.0}, x_init[2]);
}

void PiecewiseJerkQpSolver::CalculateKernel(std::vector<c_float>* P_data,
                                            std::vector<c_int>* P_indices,
                                            std::vector<c_int>* P_indptr) {
  QCHECK_NOTNULL(P_data);
  QCHECK_NOTNULL(P_indices);
  QCHECK_NOTNULL(P_indptr);

  QCHECK(P_data->empty());
  QCHECK(P_indices->empty());
  QCHECK(P_indptr->empty());

  constexpr double kEpsilon = 1e-6;

  P_indptr->push_back(0);
  int c = 0;
  for (int i = 0; i < total_num_of_var_; ++i) {
    double w = 0.0;
    if (i < num_knots_) {
      QCHECK_LT(i, weight_.x_w.size());
      w = weight_.x_w[i];
    } else if (i < 2 * num_knots_) {
      w = weight_.dx_w[i - num_knots_];
    } else if (i < 3 * num_knots_) {
      w = weight_.ddx_w[i - 2 * num_knots_];
    } else if (i < 4 * num_knots_) {
      w = weight_.dddx_w[i - 3 * num_knots_];
    } else if (i < x_upper_slack_start_idx) {
      w = weight_.x_lower_slack_w[i - x_lower_slack_start_idx];
    } else if (i < dx_lower_slack_start_idx) {
      w = weight_.x_upper_slack_w[i - x_upper_slack_start_idx];
    } else if (i < dx_upper_slack_start_idx) {
      w = weight_.dx_lower_slack_w[i - dx_lower_slack_start_idx];
    } else if (i < ddx_lower_slack_start_idx) {
      w = weight_.dx_upper_slack_w[i - dx_upper_slack_start_idx];
    } else if (i < ddx_upper_slack_start_idx) {
      w = weight_.ddx_lower_slack_w[i - ddx_lower_slack_start_idx];
    } else {
      w = weight_.ddx_upper_slack_w[i - ddx_upper_slack_start_idx];
    }

    if (std::fabs(w) > kEpsilon) {
      P_data->push_back(w);
      P_indices->push_back(i);
      ++c;
    }
    P_indptr->push_back(c);
  }
}

void PiecewiseJerkQpSolver::CalculateOffset(std::vector<c_float>* q) {
  QCHECK_NOTNULL(q);
  QCHECK(q->empty());
  *q = offset_;
}

void PiecewiseJerkQpSolver::CalculateAffineConstraint(
    std::vector<c_float>* A_data, std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr, std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  QCHECK_NOTNULL(A_data);
  QCHECK_NOTNULL(A_indices);
  QCHECK_NOTNULL(A_indptr);
  QCHECK_NOTNULL(lower_bounds);
  QCHECK_NOTNULL(upper_bounds);

  QCHECK(A_data->empty());
  QCHECK(A_indices->empty());
  QCHECK(A_indptr->empty());
  QCHECK(lower_bounds->empty());
  QCHECK(upper_bounds->empty());

  AddIntrinsicConstraint();

  A_indptr->push_back(0);
  for (int i = 0; i < constraints_data_cols_.size(); ++i) {
    A_data->insert(A_data->end(), constraints_data_cols_[i].begin(),
                   constraints_data_cols_[i].end());
    A_indices->insert(A_indices->end(), constraints_indice_cols_[i].begin(),
                      constraints_indice_cols_[i].end());
    A_indptr->push_back(A_indptr->back() + constraints_data_cols_[i].size());
  }
  *lower_bounds = lower_bounds_;
  *upper_bounds = upper_bounds_;
}

void PiecewiseJerkQpSolver::GetOptimum(const OSQPWorkspace* work) {
  segments_.clear();
  segments_.reserve(num_knots_);
  for (int i = 0; i < num_knots_; ++i) {
    segments_.emplace_back(work->solution->x[i],
                           work->solution->x[num_knots_ + i],
                           work->solution->x[2 * num_knots_ + i],
                           work->solution->x[3 * num_knots_ + i], delta_t_);
    VLOG(3) << "i = " << i << ", " << work->solution->x[i] << ", "
            << work->solution->x[num_knots_ + i] << ", "
            << work->solution->x[2 * num_knots_ + i] << ", "
            << work->solution->x[3 * num_knots_ + i];
  }

  // Extract slack variables optimum result.
  int start_index = 4 * num_knots_;
  slack_variable_.x_lower_slack_variable.reserve(
      x_lower_soft_constraint_type_num_);
  for (int i = 0; i < x_lower_soft_constraint_type_num_; ++i) {
    slack_variable_.x_lower_slack_variable.push_back(
        work->solution->x[i + start_index]);
    VLOG(3) << "x_lower_slack[" << i
            << "]: " << work->solution->x[i + start_index];
  }

  start_index += x_lower_soft_constraint_type_num_;
  slack_variable_.x_upper_slack_variable.reserve(
      x_upper_soft_constraint_type_num_);
  for (int i = 0; i < x_upper_soft_constraint_type_num_; ++i) {
    slack_variable_.x_upper_slack_variable.push_back(
        work->solution->x[i + start_index]);
    VLOG(3) << "x_upper_slack[" << i
            << "]: " << work->solution->x[i + start_index];
  }

  start_index += x_upper_soft_constraint_type_num_;
  slack_variable_.dx_lower_slack_variable.reserve(
      dx_lower_soft_constraint_type_num_);
  for (int i = 0; i < dx_lower_soft_constraint_type_num_; ++i) {
    slack_variable_.dx_lower_slack_variable.push_back(
        work->solution->x[i + start_index]);
    VLOG(3) << "dx_lower_slack[" << i
            << "]: " << work->solution->x[i + start_index];
  }

  start_index += dx_lower_soft_constraint_type_num_;
  slack_variable_.dx_upper_slack_variable.reserve(
      dx_upper_soft_constraint_type_num_);
  for (int i = 0; i < dx_upper_soft_constraint_type_num_; ++i) {
    slack_variable_.dx_upper_slack_variable.push_back(
        work->solution->x[i + start_index]);
    VLOG(3) << "dx_upper_slack[" << i
            << "]: " << work->solution->x[i + start_index];
  }

  start_index += dx_upper_soft_constraint_type_num_;
  slack_variable_.ddx_lower_slack_variable.reserve(
      ddx_lower_soft_constraint_type_num_);
  for (int i = 0; i < ddx_lower_soft_constraint_type_num_; ++i) {
    slack_variable_.ddx_lower_slack_variable.push_back(
        work->solution->x[i + start_index]);
    VLOG(3) << "ddx_lower_slack[" << i
            << "]: " << work->solution->x[i + start_index];
  }

  start_index += ddx_lower_soft_constraint_type_num_;
  slack_variable_.ddx_upper_slack_variable.reserve(
      ddx_upper_soft_constraint_type_num_);
  for (int i = 0; i < ddx_upper_soft_constraint_type_num_; ++i) {
    slack_variable_.ddx_upper_slack_variable.push_back(
        work->solution->x[i + start_index]);
    VLOG(3) << "ddx_upper_slack[" << i
            << "]: " << work->solution->x[i + start_index];
  }
}

std::array<double, 4> PiecewiseJerkQpSolver::Evaluate(double t) {
  QCHECK_GT(segments_.size(), 1);
  QCHECK_LE(t, delta_t_ * (segments_.size() - 1));
  const int idx = static_cast<int>(t / delta_t_);
  const double dt = t - idx * delta_t_;
  VLOG(3) << "idx: " << idx << ", dt: " << dt << ", t: " << t;
  return segments_[idx].Evaluate(dt);
}

double PiecewiseJerkQpSolver::GetSlackResult(int order, BoundType bound_type,
                                             int index) const {
  QCHECK(order == 0 || order == 1 || order == 2);
  double slack_ret = 0.0;
  if (order == 0) {
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(index, x_lower_soft_constraint_type_num_);
      slack_ret = slack_variable_.x_lower_slack_variable[index];
    } else {
      QCHECK_LT(index, x_upper_soft_constraint_type_num_);
      slack_ret = slack_variable_.x_upper_slack_variable[index];
    }
  }
  if (order == 1) {
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(index, dx_lower_soft_constraint_type_num_);
      slack_ret = slack_variable_.dx_lower_slack_variable[index];
    } else {
      QCHECK_LT(index, dx_upper_soft_constraint_type_num_);
      slack_ret = slack_variable_.dx_upper_slack_variable[index];
    }
  }
  if (order == 2) {
    if (bound_type == BoundType::LOWER_BOUND) {
      QCHECK_LT(index, ddx_lower_soft_constraint_type_num_);
      slack_ret = slack_variable_.ddx_lower_slack_variable[index];
    } else {
      QCHECK_LT(index, ddx_upper_soft_constraint_type_num_);
      slack_ret = slack_variable_.ddx_upper_slack_variable[index];
    }
  }
  return slack_ret;
}

absl::Status PiecewiseJerkQpSolver::OptimizeWithOsqp(
    int kernel_dim, int num_affine_constraint, std::vector<c_float>& P_data,
    std::vector<c_int>& P_indices,                                  // NOLINT
    std::vector<c_int>& P_indptr, std::vector<c_float>& A_data,     // NOLINT
    std::vector<c_int>& A_indices, std::vector<c_int>& A_indptr,    // NOLINT
    std::vector<c_float>& lower_bounds,                             // NOLINT
    std::vector<c_float>& upper_bounds,                             // NOLINT
    std::vector<c_float>& q, OSQPData* data, OSQPWorkspace** work,  // NOLINT
    OSQPSettings* settings) {
  // Define Solver settings as default
  osqp_set_default_settings(settings);
  settings->alpha = 1.6;  // Change alpha parameter
  settings->eps_abs = 1.0e-3;
  settings->eps_rel = 1.0e-3;

  settings->check_termination = 100;

  constexpr int kMaxIteration = 2000;
  settings->max_iter = kMaxIteration;
  settings->polish = true;
  settings->verbose = FLAGS_planner_enable_piecewise_jerk_qp_solver_debug;

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  QCHECK_EQ(upper_bounds.size(), lower_bounds.size());

  *work = osqp_setup(data, settings);

  // Solve problem
  osqp_solve(*work);
  if ((*work) == nullptr) {
    return absl::InternalError("OSQPWorkspace is nullptr.");
  }

  const int iters = (*work)->info->iter;
  constexpr int kCheckNum = 1000;
  if (iters >= kCheckNum) {
    QEVENT_EVERY_N_SECONDS(
        "ping", absl::StrCat("osqp_iters_reached_", std::to_string(kCheckNum)),
        1.0, [&](QEvent* qevent) { qevent->AddField("osqp_iter", iters); });
  }

  const auto status_val = (*work)->info->status_val;
  if (FLAGS_planner_use_osqp_max_iteration_result &&
      status_val == OSQP_MAX_ITER_REACHED) {
    // Usually, it doesn't trigger often.
    QLOG_EVERY_N_SEC(INFO, 1.0) << "Use max iteration result.";
    QEVENT_EVERY_N_SECONDS(
        "ping", "osqp_max_iter_reached", 1.0,
        [&](QEvent* qevent) { qevent->AddField("max_iter", kMaxIteration); });
    return absl::OkStatus();
  }

  const auto status_info = OsqpReturnStatusToString(status_val);
  if (status_val != OSQP_SOLVED && status_val != OSQP_SOLVED_INACCURATE) {
    const auto err_msg =
        absl::StrCat("Fail to solve problem with OSQP, reason: ", status_info);
    QLOG(WARNING) << err_msg;
    QLOG(INFO) << "kernel_dim = " << kernel_dim;
    QLOG(INFO) << "num_affine_constraint = " << num_affine_constraint;
    QLOG(INFO) << "status = " << status_val;
    return absl::InternalError(err_msg);
  }

  return absl::OkStatus();
}

absl::Status PiecewiseJerkQpSolver::Optimize() {
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  std::vector<c_float> q;
  CalculateOffset(&q);

  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  CalculateKernel(&P_data, &P_indices, &P_indptr);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  const absl::Status status =
      OptimizeWithOsqp(total_num_of_var_, curr_num_constraints_, P_data,
                       P_indices, P_indptr, A_data, A_indices, A_indptr,
                       lower_bounds, upper_bounds, q, data, &work_, settings);
  if (status.ok()) {
    GetOptimum(work_);
  }

  // TODO(All): realize warm start.
  osqp_cleanup(work_);
  free(data->A);
  free(data->P);
  free(data);
  free(settings);

  return status;
}

}  // namespace qcraft
