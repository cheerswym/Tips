#include "onboard/control/math/mpc_solver.h"

#include <algorithm>
#include <memory>
#include <string>

#include "absl/strings/str_cat.h"
#include "onboard/lite/logging.h"
#include "onboard/math/qp/osqp_solver.h"
#include "onboard/math/qp/qpoases_solver.h"

namespace qcraft {
namespace control {
namespace math {

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// discrete linear predictive control solver, with control format
// x(i + 1) = A[i] * x(i) + B * u (i) + C[i]
// TODO(shijun): add a unit test.
absl::Status SolveLinearMPC(const std::vector<Matrix> &matrix_a,
                            const std::vector<Matrix> &matrix_b,
                            const std::vector<Matrix> &matrix_c,
                            const Matrix &matrix_q, const Matrix &matrix_r,
                            const Matrix &matrix_n,
                            const Matrix &matrix_input_constraint_enable,
                            const std::vector<VecXd> &matrix_input_lower,
                            const std::vector<VecXd> &matrix_input_upper,
                            const Matrix &matrix_state_constraint_enable,
                            const std::vector<VecXd> &matrix_state_lower,
                            const std::vector<VecXd> &matrix_state_upper,
                            const VecXd &matrix_initial_state,
                            const std::vector<VecXd> &state_reference,
                            const std::vector<VecXd> &input_reference,
                            std::vector<VecXd> *control,
                            std::vector<VecXd> *control_unconstrained) {
  if (matrix_a[0].rows() != matrix_a[0].cols() ||
      matrix_b[0].rows() != matrix_a[0].rows() ||
      matrix_a.size() != matrix_b.size() ||
      matrix_a.size() != matrix_c.size() ||
      matrix_a[0].rows() != matrix_c[0].rows() ||
      matrix_q.rows() != matrix_q.cols() ||
      matrix_n.rows() != matrix_n.cols() ||
      matrix_q.rows() != matrix_n.rows() ||
      matrix_r.rows() != matrix_r.cols() ||
      matrix_r.rows() != matrix_input_upper[0].size() ||
      matrix_input_lower[0].size() != matrix_input_upper[0].size() ||
      matrix_input_constraint_enable.cols() != matrix_input_lower[0].size() ||
      matrix_input_constraint_enable.rows() != matrix_input_lower[0].size() ||
      matrix_state_constraint_enable.cols() != matrix_a[0].rows() ||
      matrix_state_constraint_enable.rows() != matrix_a[0].rows() ||
      matrix_state_lower[0].size() != matrix_state_upper[0].size() ||
      state_reference.size() != input_reference.size() ||
      matrix_input_lower.size() != state_reference.size() ||
      matrix_input_upper.size() != state_reference.size() ||
      matrix_state_lower.size() != state_reference.size() ||
      matrix_state_upper.size() != state_reference.size()) {
    const std::string error_msg =
        "One or more matrices have incompatible dimensions. Aborting.";
    return absl::AbortedError(error_msg);
  }

  const size_t horizon = static_cast<size_t>(state_reference.size());
  const size_t state_num = matrix_a[0].rows();
  const size_t input_num = matrix_b[0].cols();

  // Update augment state reference matrix_t
  VecXd matrix_t = VecXd::Zero(state_num * horizon);
  for (size_t j = 0; j < horizon; ++j) {
    matrix_t.block(j * state_num, 0, state_num, 1) = state_reference[j];
  }

  // Update augment input reference matrix_u
  VecXd matrix_u = VecXd::Zero(input_num * horizon);
  for (size_t j = 0; j < horizon; ++j) {
    matrix_u.block(j * input_num, 0, input_num, 1) = input_reference[j];
  }

  // Initialize augment control_output matrix_v which is used to store
  // solved result;
  VecXd matrix_v = VecXd::Zero(input_num * horizon);

  //  Convert formula: x(i + 1) = A[i] * x(i) + B * u (i) + C[i] to below
  //  X[i] = M[i] + K[i] * U[i] + CC[i]
  // Update augment matrix_m
  Matrix matrix_m = Matrix::Zero(state_num * horizon, 1);
  matrix_m.block(0, 0, state_num, 1) = matrix_a[0] * matrix_initial_state;
  for (size_t i = 1; i < horizon; ++i) {
    matrix_m.block(i * state_num, 0, state_num, 1) =
        matrix_a[i] * matrix_m.block((i - 1) * state_num, 0, state_num, 1);
  }

  // Update augment matrix_k
  Matrix matrix_k = Matrix::Zero(state_num * horizon, input_num * horizon);
  matrix_k.block(0, 0, state_num, input_num) = matrix_b[0];
  for (size_t r = 1; r < horizon; ++r) {
    matrix_k.block(r * state_num, 0, state_num, r * input_num) =
        matrix_a[r] *
        matrix_k.block((r - 1) * state_num, 0, state_num, r * input_num);
    matrix_k.block(r * state_num, r * input_num, state_num, input_num) =
        matrix_b[r];
  }

  // Compute matrix_cc
  Matrix matrix_cc = Matrix::Zero(horizon * state_num, 1);
  matrix_cc.block(0, 0, state_num, 1) = matrix_c[0];
  for (size_t i = 1; i < horizon; ++i) {
    matrix_cc.block(i * state_num, 0, state_num, 1) =
        matrix_a[i] * matrix_cc.block((i - 1) * state_num, 0, state_num, 1) +
        matrix_c[i];
  }

  // Update augment  matrix_qq, matrix_rr
  Matrix matrix_qq = Matrix::Zero(matrix_k.rows(), matrix_k.rows());
  Matrix matrix_rr = Matrix::Zero(matrix_k.cols(), matrix_k.cols());
  for (size_t i = 0; i < horizon; ++i) {
    matrix_qq.block(i * state_num, i * state_num, state_num, state_num) =
        matrix_q;
    matrix_rr.block(i * input_num, i * input_num, input_num, input_num) =
        matrix_r;
  }

  // implement a simplistic terminal cost to suppress oscillation
  matrix_qq.block((horizon - 1) * state_num, (horizon - 1) * state_num,
                  state_num, state_num) =
      matrix_n.transpose() *
      matrix_qq.block((horizon - 1) * state_num, (horizon - 1) * state_num,
                      state_num, state_num) *
      matrix_n;

  // Update augment matrix_state_en, matrix_state_ll, matrix_state_uu
  Matrix matrix_state_en =
      Matrix::Zero(state_num * horizon, state_num * horizon);
  VecXd matrix_state_ll = VecXd::Zero(horizon * state_num);
  VecXd matrix_state_uu = VecXd::Zero(horizon * state_num);
  for (size_t i = 0; i < horizon; ++i) {
    matrix_state_en.block(i * state_num, i * state_num, state_num, state_num) =
        matrix_state_constraint_enable;
    matrix_state_ll.block(i * state_num, 0, state_num, 1) =
        matrix_state_lower[i];
    matrix_state_uu.block(i * state_num, 0, state_num, 1) =
        matrix_state_upper[i];
  }

  // Update augment matrix_inequality_state_constraint_ll,
  // matrix_inequality_state_constraint_uu
  Matrix matrix_inequality_state_constraint_ll =
      Matrix::Zero(horizon * state_num, 1);
  Matrix matrix_inequality_state_constraint_uu =
      Matrix::Zero(horizon * state_num, 1);
  matrix_inequality_state_constraint_ll = matrix_state_en * matrix_k;
  matrix_inequality_state_constraint_uu = -matrix_state_en * matrix_k;

  // Update augment matrix_inequality_state_boundary_ll,
  // matrix_inequality_state_boundary_uu
  VecXd matrix_inequality_state_boundary_ll = VecXd::Zero(horizon * state_num);
  VecXd matrix_inequality_state_boundary_uu = VecXd::Zero(horizon * state_num);
  matrix_inequality_state_boundary_ll =
      matrix_state_ll - matrix_state_en * (matrix_m + matrix_cc);
  matrix_inequality_state_boundary_uu =
      -matrix_state_uu + matrix_state_en * (matrix_m + matrix_cc);

  // Update augment input_ll, matrix_input_uu,
  // matrix_inequality_input_constraint_ll,
  // matrix_inequality_input_constraint_uu
  VecXd matrix_input_ll = VecXd::Zero(horizon * input_num);
  VecXd matrix_input_uu = VecXd::Zero(horizon * input_num);

  Matrix matrix_inequality_input_constraint_ll =
      Matrix::Zero(matrix_input_ll.size(), matrix_input_ll.size());
  Matrix matrix_inequality_input_constraint_uu =
      Matrix::Zero(matrix_input_uu.size(), matrix_input_uu.size());

  for (size_t i = 0; i < horizon; ++i) {
    matrix_input_ll.block(i * input_num, 0, input_num, 1) =
        matrix_input_lower[i];
    matrix_input_uu.block(i * input_num, 0, input_num, 1) =
        matrix_input_upper[i];
    matrix_inequality_input_constraint_ll.block(i * input_num, i * input_num,
                                                input_num, input_num) =
        matrix_input_constraint_enable;
    matrix_inequality_input_constraint_uu.block(i * input_num, i * input_num,
                                                input_num, input_num) =
        -matrix_input_constraint_enable;
  }

  // Update augment matrix_inequality_input_boundary_ll,
  // matrix_inequality_input_boundary_uu
  VecXd matrix_inequality_input_boundary_ll = matrix_input_ll;
  VecXd matrix_inequality_input_boundary_uu = -matrix_input_uu;

  // Update matrix_m1, matrix_m2, convert MPC problem to QP problem
  // m1 = k^T * Q * k + R,   m2 = k^T * Q * (m + cc - t) - R * u
  // t: state reference matrix  u: input reference matrix
  const Matrix matrix_m1 = static_cast<Matrix>(
      matrix_k.transpose() * matrix_qq * matrix_k + matrix_rr);
  const VecXd matrix_m2 = static_cast<Matrix>(
      matrix_k.transpose() * matrix_qq * (matrix_m + matrix_cc - matrix_t) -
      matrix_rr * matrix_u);
  const VecXd matrix_control_unconstrained = -matrix_m1.inverse() * matrix_m2;

  // Format in qp_solver

  //    min_x  : q(x) = 0.5 * x^T * m1 * x  + x^T m2
  //    with respect to:  n1 * x = n2 (equality constraint)
  //                      n3 * x >= n4 (inequality constraint)
  //    where, n1: matrix_equality_constraint
  //           n2: matrix_equality_boundary
  //           n3: matrix_inequality_constraint
  //           n4: matrix_inequality_boundary

  // Process equality constraints (inactive).
  Matrix matrix_equality_constraint = Matrix::Zero(
      matrix_input_ll.size() + matrix_input_uu.size(), matrix_input_ll.size());
  VecXd matrix_equality_boundary =
      VecXd::Zero(matrix_input_ll.size() + matrix_input_uu.size());

  // Process inequality constrtaints
  Matrix matrix_inequality_constraint;
  VecXd matrix_inequality_boundary;

  if (matrix_state_constraint_enable.isZero()) {
    matrix_inequality_constraint =
        Matrix::Zero(matrix_inequality_input_constraint_ll.rows() +
                         matrix_inequality_input_constraint_uu.rows(),
                     matrix_inequality_input_constraint_ll.cols());
    matrix_inequality_constraint << matrix_inequality_input_constraint_ll,
        matrix_inequality_input_constraint_uu;

    matrix_inequality_boundary =
        VecXd::Zero(matrix_inequality_input_boundary_ll.size() +
                    matrix_inequality_input_boundary_uu.size());
    matrix_inequality_boundary << matrix_inequality_input_boundary_ll,
        matrix_inequality_input_boundary_uu;
  } else {
    matrix_inequality_constraint =
        Matrix::Zero(matrix_inequality_input_constraint_ll.rows() +
                         matrix_inequality_input_constraint_uu.rows() +
                         matrix_inequality_state_constraint_ll.rows() +
                         matrix_inequality_state_constraint_uu.rows(),
                     matrix_inequality_input_constraint_ll.cols());
    matrix_inequality_constraint << matrix_inequality_input_constraint_ll,
        matrix_inequality_input_constraint_uu,
        matrix_inequality_state_constraint_ll,
        matrix_inequality_state_constraint_uu;

    matrix_inequality_boundary =
        VecXd::Zero(matrix_inequality_input_boundary_ll.rows() +
                    matrix_inequality_input_boundary_uu.rows() +
                    matrix_inequality_state_boundary_ll.rows() +
                    matrix_inequality_state_boundary_uu.rows());
    matrix_inequality_boundary << matrix_inequality_input_boundary_ll,
        matrix_inequality_input_boundary_uu,
        matrix_inequality_state_boundary_ll,
        matrix_inequality_state_boundary_uu;
  }

  // Method 1: QPOASES
  //   std::unique_ptr<qcraft::DenseQpSolver> qp_solver(new
  //   qcraft::QpoasesSolver(
  //       matrix_m1, matrix_m2, matrix_equality_constraint,
  //       matrix_equality_boundary, matrix_inequality_constraint,
  //       matrix_inequality_boundary));
  //   auto result = qp_solver->Solve();
  //   if (!result) {
  //     QLOG(ERROR) << "Linear MPC solver failed";
  //     return false;
  //   }
  //   matrix_v = qp_solver->x();

  // Method 2: osqp
  std::unique_ptr<qcraft::SparseQpSolver> qp_solver;
  qp_solver = std::make_unique<qcraft::OsqpSolver>(
      matrix_m1.sparseView(), matrix_m2,
      matrix_equality_constraint.sparseView(), matrix_equality_boundary,
      matrix_inequality_constraint.sparseView(), matrix_inequality_boundary);
  const auto status = qp_solver->Solve();

  if (!status.ok()) {
    return absl::InternalError(status.ToString());
  }

  matrix_v = qp_solver->x();

  for (size_t i = 0; i < horizon; ++i) {
    (*control)[i] = matrix_v.block(i * input_num, 0, input_num, 1);

    (*control_unconstrained)[i] =
        matrix_control_unconstrained.block(i * input_num, 0, input_num, 1);
  }
  return absl::OkStatus();
}

absl::Status SolveLinearMPC(const std::vector<Matrix> &matrix_a,
                            const Matrix &matrix_b,
                            const std::vector<Matrix> &matrix_c,
                            const Matrix &matrix_q, const Matrix &matrix_r,
                            const Matrix &matrix_n,
                            const Matrix &matrix_input_constraint_enable,
                            const std::vector<VecXd> &matrix_input_lower,
                            const std::vector<VecXd> &matrix_input_upper,
                            const Matrix &matrix_state_constraint_enable,
                            const std::vector<VecXd> &matrix_state_lower,
                            const std::vector<VecXd> &matrix_state_upper,
                            const VecXd &matrix_initial_state,
                            const std::vector<VecXd> &state_reference,
                            const std::vector<VecXd> &input_reference,
                            std::vector<VecXd> *control,
                            std::vector<VecXd> *control_unconstrained) {
  std::vector<Matrix> matrix_bt;
  matrix_bt.assign(matrix_a.size(), matrix_b);

  return SolveLinearMPC(
      matrix_a, matrix_bt, matrix_c, matrix_q, matrix_r, matrix_n,
      matrix_input_constraint_enable, matrix_input_lower, matrix_input_upper,
      matrix_state_constraint_enable, matrix_state_lower, matrix_state_upper,
      matrix_initial_state, state_reference, input_reference, control,
      control_unconstrained);
}

absl::Status SolveLinearMPC(const std::vector<Matrix> &matrix_a,
                            const Matrix &matrix_b, const Matrix &matrix_c,
                            const Matrix &matrix_q, const Matrix &matrix_r,
                            const Matrix &matrix_n,
                            const Matrix &matrix_input_constraint_enable,
                            const std::vector<VecXd> &matrix_input_lower,
                            const std::vector<VecXd> &matrix_input_upper,
                            const Matrix &matrix_state_constraint_enable,
                            const std::vector<VecXd> &matrix_state_lower,
                            const std::vector<VecXd> &matrix_state_upper,
                            const VecXd &matrix_initial_state,
                            const std::vector<VecXd> &state_reference,
                            const std::vector<VecXd> &input_reference,
                            std::vector<VecXd> *control,
                            std::vector<VecXd> *control_unconstrained) {
  std::vector<Matrix> matrix_ct;
  matrix_ct.assign(matrix_a.size(), matrix_c);
  std::vector<Matrix> matrix_bt;
  matrix_bt.assign(matrix_a.size(), matrix_b);

  return SolveLinearMPC(
      matrix_a, matrix_bt, matrix_ct, matrix_q, matrix_r, matrix_n,
      matrix_input_constraint_enable, matrix_input_lower, matrix_input_upper,
      matrix_state_constraint_enable, matrix_state_lower, matrix_state_upper,
      matrix_initial_state, state_reference, input_reference, control,
      control_unconstrained);
}

}  // namespace math
}  // namespace control
}  // namespace qcraft
