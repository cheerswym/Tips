#include "onboard/math/qp/osqp_solver.h"

#include <memory>
#include <vector>

namespace qcraft {

OsqpSolver::OsqpSolver(const SMatXd &A, const VecXd &b, const SMatXd &G,
                       const VecXd &g, const SMatXd &H, const VecXd &h)
    : SparseQpSolver(A, b, G, g, H, h) {}

bool OsqpSolver::Solve() {
  auto settings = std::make_unique<OSQPSettings>();
  osqp_set_default_settings(settings.get());
  settings->alpha = 1.0;
  settings->eps_abs = 1e-7;
  settings->eps_rel = 1e-7;
  settings->verbose = false;
  return Solve(*settings);
}

bool OsqpSolver::Solve(const OSQPSettings &settings) {
  // TODO(Fang) change osqp/osqp_configure.h to make osqp type c_int be int
  // instead of long long. Then this copying for A can be avoided (because
  // SMatXd::StorageIndex is int).
  A_.uncompress();
  std::vector<c_float> A_values;
  std::vector<c_int> A_inner_indices;
  std::vector<c_int> A_outer_starts;
  A_values.reserve(A().nonZeros());
  A_inner_indices.reserve(A_values.size());
  A_outer_starts.reserve(nx() + 1);
  A_outer_starts.push_back(0);
  for (int i = 0; i < nx(); ++i) {
    for (int j = 0; j < A().innerNonZeroPtr()[i]; ++j) {
      A_values.push_back(A().valuePtr()[A().outerIndexPtr()[i] + j]);
      A_inner_indices.push_back(
          A().innerIndexPtr()[A().outerIndexPtr()[i] + j]);
    }
    A_outer_starts.push_back(A_values.size());
  }
  QCHECK_EQ(A_values.size(), A().nonZeros());

  G_.uncompress();
  H_.uncompress();
  std::vector<c_float> constraint_values;
  std::vector<c_int> constraint_inner_indices;
  std::vector<c_int> constraint_outer_starts;
  constraint_values.reserve(G().nonZeros() + H().nonZeros());
  constraint_inner_indices.reserve(constraint_values.size());
  constraint_outer_starts.reserve(nx() + 1);
  constraint_outer_starts.push_back(0);
  for (int i = 0; i < nx(); ++i) {
    for (int j = 0; j < G().innerNonZeroPtr()[i]; ++j) {
      constraint_values.push_back(G().valuePtr()[G().outerIndexPtr()[i] + j]);
      constraint_inner_indices.push_back(
          G().innerIndexPtr()[G().outerIndexPtr()[i] + j]);
    }
    for (int j = 0; j < H().innerNonZeroPtr()[i]; ++j) {
      constraint_values.push_back(H().valuePtr()[H().outerIndexPtr()[i] + j]);
      constraint_inner_indices.push_back(
          H().innerIndexPtr()[H().outerIndexPtr()[i] + j] + ng());
    }
    constraint_outer_starts.push_back(constraint_values.size());
  }
  QCHECK_EQ(constraint_values.size(), G().nonZeros() + H().nonZeros());

  constexpr double kConstraintUpperBound = 1e10;  // Essentially unbounded.
  VecXd l = VecXd::Zero(ng() + nh());
  VecXd u = VecXd::Zero(ng() + nh());
  l.segment(0, ng()) = g();
  u.segment(0, ng()) = g();
  l.segment(ng(), nh()) = h();
  u.segment(ng(), nh()) = VecXd::Constant(nh(), kConstraintUpperBound);

  auto data = std::make_unique<OSQPData>();
  data->n = nx();
  data->m = ng() + nh();
  data->P = csc_matrix(data->n, data->n, A_values.size(), A_values.data(),
                       A_inner_indices.data(), A_outer_starts.data());
  data->q = const_cast<double *>(b().data());
  data->A = csc_matrix(
      data->m, data->n, constraint_values.size(), constraint_values.data(),
      constraint_inner_indices.data(), constraint_outer_starts.data());
  data->l = const_cast<double *>(l.data());
  data->u = const_cast<double *>(u.data());

  OSQPWorkspace *work =
      osqp_setup(data.get(), const_cast<OSQPSettings *>(&settings));

  const int result = osqp_solve(work);
  VLOG(3) << "OSQP solve result: " << result
          << " solution status: " << work->info->status_val << " ("
          << work->info->status << ") iterations: " << work->info->iter
          << " time: " << work->info->run_time << "s";

  x_ = VecXd::Zero(nx());
  for (int i = 0; i < nx(); ++i) x_[i] = work->solution->x[i];

  osqp_cleanup(work);
  c_free(data->A);
  c_free(data->P);
  return true;
}

}  // namespace qcraft
