#ifndef ONBOARD_CONTROL_MATH_MPC_SOLVER_H_
#define ONBOARD_CONTROL_MATH_MPC_SOLVER_H_

#include <vector>

#include "Eigen/Core"
#include "absl/status/status.h"

namespace qcraft {
namespace control {
namespace math {

using Matrix = Eigen::MatrixXd;
using VecXd = Eigen::VectorXd;

// Convert mpc problem to QP based problem and solve.
// Add state constraints into consideration

// matrix_a: The system dynamic matrix (time-variant or time-invariant);
// matrix_b: The control matrix;
// matrix_c: The disturbance matrix;
// matrix_q: The cost matrix for control state;
// matrix_r: The cost matrix for input;
// matrix_input_constraint_enable: The matrix enables specific input
// constraints; matrix_input_lower: The lower bound control constraint matrix;
// matrix_input_upper: The upper bound control constraint matrix;
// matrix_n: The cost matrix of terminal state scale;
// matrix_state_constraint_enable: The matrix enables specific state
// constraints; matrix_state_lower: The lower bound state constraint matrix;
// matrix_state_upper: The upper bound state constraint matrix;
// matrix_initial_state: The initial state matrix;
// state_reference: The state reference vector with respect to time step;
// input_reference: The input reference vector with respect to time step;
// control: The feedback control matrix (pointer);
// control_unconstraint: The optimal solution without considering constraints;

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
                            std::vector<VecXd> *control_unconstrained);

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
                            std::vector<VecXd> *control_unconstrained);

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
                            std::vector<VecXd> *control_unconstrained);

}  // namespace math
}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_MATH_MPC_SOLVER_H_
