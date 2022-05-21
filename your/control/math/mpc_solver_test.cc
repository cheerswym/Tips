/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "onboard/control/math/mpc_solver.h"

#include "gtest/gtest.h"
// IWYU pragma: no_include  <ext/alloc_traits.h>

namespace qcraft {
namespace control {
namespace math {
namespace {

TEST(MPCSolverTest, MPC) {
  const int states = 4;
  int controls = 2;
  const int horizon = 2;

  // Test 0

  Eigen::MatrixXd a(states, states);
  a << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1;
  std::vector<Eigen::MatrixXd> A(horizon, a);

  Eigen::MatrixXd B(states, controls);
  B << 0, 1, 0, 0, 1, 0, 0, 1;

  Eigen::MatrixXd C(states, 1);
  C << 0, 0, 0, 0.1;

  Eigen::MatrixXd Q(states, states);
  Q << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::MatrixXd R(controls, controls);
  R << 1, 0, 0, 1;

  Eigen::VectorXd lower_bound(controls);
  lower_bound << -10, -10;

  Eigen::VectorXd upper_bound(controls);
  upper_bound << 10, 10;

  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(states, states);

  Eigen::VectorXd initial_state(states);
  initial_state << 0, 0, 0, 0;

  Eigen::VectorXd reference_state(states);
  reference_state << 200, 200, 0, 0;

  std::vector<Eigen::VectorXd> state_reference(horizon, reference_state);

  Eigen::VectorXd control_matrix(controls);
  control_matrix << 0, 0;

  std::vector<Eigen::VectorXd> input_reference(horizon, control_matrix);

  std::vector<Eigen::VectorXd> control(horizon, control_matrix);
  std::vector<Eigen::VectorXd> control_unconstrained(horizon, control_matrix);

  Eigen::MatrixXd matrix_input_constraint_enable =
      Eigen::MatrixXd::Identity(controls, controls);

  Eigen::MatrixXd matrix_state_constraint_enable =
      Eigen::MatrixXd::Zero(states, states);

  Eigen::VectorXd matrix_state_lower = Eigen::VectorXd::Zero(states);
  Eigen::VectorXd matrix_state_upper = Eigen::VectorXd::Zero(states);

  std::vector<Eigen::VectorXd> input_lower(horizon, lower_bound);
  std::vector<Eigen::VectorXd> input_upper(horizon, upper_bound);
  std::vector<Eigen::VectorXd> state_lower(horizon, matrix_state_lower);
  std::vector<Eigen::VectorXd> state_upper(horizon, matrix_state_upper);
  for (unsigned int i = 0; i < control.size(); ++i) {
    for (unsigned int i = 1; i < control.size(); ++i) {
      control[i - 1] = control[i];
    }
    control[horizon - 1] = control_matrix;
    SolveLinearMPC(A, B, C, Q, R, N, matrix_input_constraint_enable,
                   input_lower, input_upper, matrix_state_constraint_enable,
                   state_lower, state_upper, initial_state, state_reference,
                   input_reference, &control, &control_unconstrained)
        .IgnoreError();
    EXPECT_FLOAT_EQ(upper_bound(0), control[0](0));
  }

  // Test 1

  controls = 1;

  Eigen::MatrixXd B1(states, controls);
  B1 << 0, 0, 1, 0;

  Eigen::MatrixXd R1(controls, controls);
  R1 << 1;

  Eigen::VectorXd lower_bound1(controls);
  lower_bound1 << -5;

  Eigen::VectorXd upper_bound1(controls);
  upper_bound1 << 5;

  Eigen::VectorXd initial_state1(states);
  initial_state1 << 100, 100, 0, 0;

  Eigen::VectorXd reference_state1(states);
  reference_state1 << 0, 0, 0, 0;

  std::vector<Eigen::VectorXd> reference1(horizon, reference_state1);

  Eigen::VectorXd control_matrix1(controls);
  control_matrix1 << 0;
  std::vector<Eigen::VectorXd> input_reference1(horizon, control_matrix1);
  std::vector<Eigen::VectorXd> control1(horizon, control_matrix1);

  Eigen::MatrixXd control_gain_matrix1(controls, states);
  control_gain_matrix1 << 0, 0, 0, 0;
  std::vector<Eigen::MatrixXd> control_gain1(horizon, control_gain_matrix1);

  Eigen::VectorXd addition_gain_matrix1(controls);
  addition_gain_matrix1 << 0;
  std::vector<Eigen::VectorXd> control_unconstrained1(horizon,
                                                      addition_gain_matrix1);

  Eigen::MatrixXd matrix_input_constraint_enable1 =
      Eigen::MatrixXd::Identity(controls, controls);

  std::vector<Eigen::VectorXd> input_lower1(horizon, lower_bound1);
  std::vector<Eigen::VectorXd> input_upper1(horizon, upper_bound1);

  for (unsigned int i = 0; i < control1.size(); ++i) {
    for (unsigned int i = 1; i < control1.size(); ++i) {
      control1[i - 1] = control1[i];
    }
    SolveLinearMPC(A, B1, C, Q, R1, N, matrix_input_constraint_enable1,
                   input_lower1, input_upper1, matrix_state_constraint_enable,
                   state_lower, state_upper, initial_state1, reference1,
                   input_reference1, &control1, &control_unconstrained1)
        .IgnoreError();
    EXPECT_FLOAT_EQ(lower_bound1(0), control1[0](0));
  }
}

}  // namespace
}  // namespace math
}  // namespace control
}  // namespace qcraft
