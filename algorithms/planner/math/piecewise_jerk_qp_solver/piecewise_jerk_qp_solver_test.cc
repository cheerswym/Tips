#include "onboard/planner/math/piecewise_jerk_qp_solver/piecewise_jerk_qp_solver.h"

#include "gtest/gtest.h"
#include "onboard/lite/logging.h"

namespace qcraft {
namespace {

TEST(PiecewiseJerkQpSolverTest, TestQpSolverWithoutSlack) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;
  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(kElementNum, kDeltaT);
  constexpr double ref_pos = 200.0;

  auto start_time = absl::Now();

  // Add kernel
  constexpr double x_w = 1.0;
  constexpr double dx_w = 0.0001;
  constexpr double ddx_w = 10.0;
  constexpr double dddx_w = 10.0;
  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel(0, i, ref_pos, x_w);
  }
  solver->AddFirstOrderDerivativeKernel(dx_w);
  solver->AddSecondOrderDerivativeKernel(ddx_w);
  solver->AddThirdOrderDerivativeKernel(dddx_w);

  // Add equality constraints
  std::array<double, 3> x_init = {0.0, 0.0, 0.0};
  solver->SetInitConditions(x_init);

  // Add inequality constraints
  for (int i = 0; i < 50; ++i) {
    constexpr double coeff = 1.0;
    // s constraint
    constexpr double x_lower_bound = 0.0;
    constexpr double x_upper_bound = 20.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{coeff}, x_lower_bound,
                                            x_upper_bound);
    // v constraint
    constexpr double dx_lower_bound = 0.0;
    constexpr double dx_upper_bound = 4.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/1, /*index_list=*/{i},
                                            /*coeff=*/{coeff}, dx_lower_bound,
                                            dx_upper_bound);
    // a constraint
    constexpr double ddx_lower_bound = -2.0;
    constexpr double ddx_upper_bound = 2.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/2, /*index_list=*/{i},
                                            /*coeff=*/{coeff}, ddx_lower_bound,
                                            ddx_upper_bound);
  }

  // Add monotonic constraints: guarantee monotonic increase of s
  solver->AddMonotonicConstraints();

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.06) {
      auto res = solver->Evaluate(t);
      QLOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                 << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }

    QLOG(INFO) << "qp solver cost time: "
               << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    QLOG(INFO) << "qp solver failed.";
  }
}

TEST(PiecewiseJerkQpSolverTest, TestWithSlackOnOneSide) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;

  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(
          kElementNum, kDeltaT, /*x_lower_soft_constraint_type_num=*/0,
          /*x_upper_soft_constraint_type_num=*/1,
          /*dx_lower_soft_constraint_type_num=*/0,
          /*dx_upper_soft_constraint_type_num=*/2,
          /*ddx_lower_soft_constraint_type_num=*/0,
          /*ddx_upper_soft_constraint_type_num=*/0);
  constexpr double ref_pos = 30.0;

  auto start_time = absl::Now();

  // Add kernel
  constexpr double x_w = 0.1;
  constexpr double dx_w = 0.0001;
  constexpr double ddx_w = 1.0;
  constexpr double dddx_w = 10.0;
  constexpr double dx_upper_slack_w = 10.0;

  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel(0, i, ref_pos, x_w);
  }
  solver->AddFirstOrderDerivativeKernel(dx_w);
  solver->AddSecondOrderDerivativeKernel(ddx_w);
  solver->AddThirdOrderDerivativeKernel(dddx_w);
  solver->AddFirstOrderSlackVarKernel(BoundType::UPPER_BOUND,
                                      /*index_list=*/{0, 1},
                                      {dx_upper_slack_w, dx_upper_slack_w});

  // Add equality constraints
  constexpr double kInitV = 6.0;

  std::array<double, 3> x_init = {0.0, kInitV, 0.0};
  solver->SetInitConditions(x_init);

  // Add inequality constraints
  for (int i = 0; i < 50; ++i) {
    constexpr double coeff = 1.0;
    // s constraint
    constexpr double x_lower_bound = 0.0;
    constexpr double x_upper_bound = 20.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{coeff}, x_lower_bound,
                                            x_upper_bound);

    // v constraint
    // The first type of speed constraint
    constexpr double dx_lower_bound_type1 = 0.0;
    constexpr double dx_upper_bound_type1 = 4.0;
    solver->AddFirstOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{coeff}, /*slack_term_index=*/0,
        BoundType::UPPER_BOUND, dx_lower_bound_type1, dx_upper_bound_type1);
    // The second type of speed constraint
    constexpr double dx_lower_bound_type2 = 0.0;
    constexpr double dx_upper_bound_type2 = 3.0;
    solver->AddFirstOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{coeff}, /*slack_term_index=*/1,
        BoundType::UPPER_BOUND, dx_lower_bound_type2, dx_upper_bound_type2);

    // a constraint
    constexpr double ddx_lower_bound = -4.0;
    constexpr double ddx_upper_bound = 4.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/2, /*index_list=*/{i},
                                            /*coeff=*/{coeff}, ddx_lower_bound,
                                            ddx_upper_bound);
  }

  // Add monotonic constraints: guarantee monotonic increase of s
  solver->AddMonotonicConstraints();

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.06) {
      auto res = solver->Evaluate(t);
      QLOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                 << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }

    QLOG(INFO) << "qp solver with slack on one side cost time: "
               << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  } else {
    QLOG(INFO) << "qp solver with slack on one side failed.";
  }
}

TEST(PiecewiseJerkQpSolverTest, TestAccelSoftConstraint) {
  constexpr int kElementNum = 50;
  constexpr double kDeltaT = 0.1;

  std::unique_ptr<PiecewiseJerkQpSolver> solver =
      std::make_unique<PiecewiseJerkQpSolver>(
          kElementNum, kDeltaT, /*x_lower_soft_constraint_type_num=*/0,
          /*x_upper_soft_constraint_type_num=*/0,
          /*dx_lower_soft_constraint_type_num=*/0,
          /*dx_upper_soft_constraint_type_num=*/0,
          /*ddx_lower_soft_constraint_type_num=*/1,
          /*ddx_upper_soft_constraint_type_num=*/0);

  // Add kernel
  constexpr double kSpeedWeight = 0.1;
  constexpr double kAccelWeight = 1.0;
  constexpr double kJerkWeight = 1.0;
  constexpr double kAccelSlackWeight = 20.0;

  constexpr double ref_speed = 10.0;
  for (int i = 0; i < kElementNum; ++i) {
    solver->AddNthOrderReferencePointKernel(/*order=*/1, i, ref_speed,
                                            kSpeedWeight);
  }
  solver->AddSecondOrderDerivativeKernel(kAccelWeight);
  solver->AddThirdOrderDerivativeKernel(kJerkWeight);
  solver->AddSecondOrderSlackVarKernel(BoundType::LOWER_BOUND,
                                       /*index_list=*/{0}, {kAccelSlackWeight});

  // Add equality constraints.
  constexpr double kInitV = 8.0;
  solver->AddNthOrderEqualityConstraint(/*order=*/0, {0}, {1.0}, 0.0);
  solver->AddNthOrderEqualityConstraint(/*order=*/1, {0}, {1.0}, kInitV);
  solver->AddNthOrderEqualityConstraint(/*order=*/1, {kElementNum - 1}, {1.0},
                                        0.0);

  // Add inequality constraints.
  for (int i = 1; i < kElementNum; ++i) {
    constexpr double coeff = 1.0;
    // Add s constraint.
    constexpr double kSLowerBound = 0.0;
    constexpr double kSUpperBound = 50.0;
    solver->AddNthOrderInequalityConstraint(/*order=*/0, /*index_list=*/{i},
                                            /*coeff=*/{coeff}, kSLowerBound,
                                            kSUpperBound);

    // Add speed constraint.
    constexpr double kSpeedLowerBound = 0.0;
    constexpr double kSpeedUpperBound = 20.0;
    solver->AddNthOrderInequalityConstraint(
        /*order=*/1, /*index_list=*/{i},
        /*coeff=*/{coeff}, kSpeedLowerBound, kSpeedUpperBound);

    // Add accel constraint.
    constexpr double kMaxAccelLowerBound = -4.0;
    constexpr double kAccelLowerBound = -2.0;
    constexpr double kAccelUpperBound = 2.0;

    /*
     *  Add accelration soft constraint.
     *
     */
    solver->AddSecondOrderInequalityConstraintWithOneSideSlack(
        /*index_list=*/{i}, /*coeff=*/{coeff}, /*slack_term_index=*/0,
        BoundType::LOWER_BOUND, kAccelLowerBound, kAccelUpperBound);

    /*
     *  Add accelration hard constraint.
     *
     */
    solver->AddNthOrderInequalityConstraint(
        /*order=*/2, /*index_list=*/{i},
        /*coeff=*/{coeff}, kMaxAccelLowerBound, kAccelUpperBound);
  }

  // Add monotonic constraints: guarantee monotonic increase of s
  solver->AddMonotonicConstraints();

  // Solve
  const auto success = solver->Optimize();

  EXPECT_TRUE(success.ok());

  if (success.ok()) {
    // Result
    for (double t = 0.0; t < 4.90; t += 0.2) {
      auto res = solver->Evaluate(t);
      QLOG(INFO) << "t[" << t << "], s[" << res[0] << "], v[" << res[1]
                 << "], a[" << res[2] << "], jerk[" << res[3] << "]";
    }
  } else {
    QLOG(INFO) << "qp solver with slack on one side failed.";
  }
}

}  // namespace
}  // namespace qcraft
