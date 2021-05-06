#include "onboard/math/qp/qpoases_solver.h"

#include <algorithm>

#include "onboard/lite/logging.h"

namespace qcraft {

QpoasesSolver::QpoasesSolver(const MatXd &A, const VecXd &b, const MatXd &G,
                             const VecXd &g, const MatXd &H, const VecXd &h)
    : DenseQpSolver(A, b, G, g, H, h) {}

bool QpoasesSolver::Solve() {
  qpOASES::Options options;
  options.enableCholeskyRefactorisation = 0;         // Disabled.
  options.enableRegularisation = qpOASES::BT_FALSE;  // Hessian unknown.
  // options.setToFast();
  options.setToReliable();
  return Solve(options);
}

bool QpoasesSolver::Solve(const qpOASES::Options &options) {
  qpOASES::QProblem problem(nx(), ng() + nh(), qpOASES::HST_UNKNOWN);
  problem.setOptions(options);
  problem.setPrintLevel(qpOASES::PL_NONE);

  constexpr double kVariableBound = 1e10;  // Essentially unbounded.
  const VecXd variable_lower_bounds = -VecXd::Ones(nx()) * kVariableBound;
  const VecXd variable_upper_bounds = VecXd::Ones(nx()) * kVariableBound;

  constexpr double kConstraintUpperBound = 1e10;  // Essentially unbounded.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      constraints(ng() + nh(), nx());
  VecXd constraint_lower_bounds = VecXd::Zero(ng() + nh());
  VecXd constraint_upper_bounds = VecXd::Zero(ng() + nh());
  constraints.block(0, 0, ng(), nx()) = G();
  constraints.block(ng(), 0, nh(), nx()) = H();
  constraint_lower_bounds.segment(0, ng()) = g();
  constraint_upper_bounds.segment(0, ng()) = g();
  constraint_lower_bounds.segment(ng(), nh()) = h();
  constraint_upper_bounds.segment(ng(), nh()) =
      VecXd::Constant(nh(), kConstraintUpperBound);

  constexpr int kMaxIters = 1000;
  int max_iters = std::max(kMaxIters, ng() + nh());

  // Passing Eigen matrix's data directly into qpOASES as arrays. This requires
  // the matrices to be row-major. It's okay for A because A is symmetric. It's
  // okay for b and variable/constraint bounds because they are vectors. The
  // constraints matrix is specifically constructed as row-major so it's okay
  // too.
  const auto result = problem.init(
      A_.data(), b_.data(), constraints.data(), variable_lower_bounds.data(),
      variable_upper_bounds.data(), constraint_lower_bounds.data(),
      constraint_upper_bounds.data(), max_iters);
  if (result != qpOASES::SUCCESSFUL_RETURN) {
    QLOG(ERROR) << "qpOASES solver failed with error: "
                << static_cast<int>(result);
    VLOG(3) << DebugString();
    return false;
  }
  if (problem.isSolved() != qpOASES::BT_TRUE) {
    QLOG(ERROR) << "qpOASES solver succeeded but the problem is not solved.";
    VLOG(3) << DebugString();
    return false;
  }

  x_ = VecXd::Zero(nx());
  problem.getPrimalSolution(x_.data());
  return true;
}

}  // namespace qcraft
