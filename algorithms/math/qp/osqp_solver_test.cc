#include "onboard/math/qp/osqp_solver.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(OsqpSolverTest, SimpleTest) {
  MatXd A = MatXd::Ones(1, 1);
  VecXd b = VecXd::Ones(1) * 5.0;
  MatXd G = MatXd::Zero(1, 1);
  VecXd g = VecXd::Zero(1);
  MatXd H = MatXd::Zero(1, 1);
  VecXd h = VecXd::Zero(1);
  OsqpSolver solver(A.sparseView(), b, G.sparseView(), g, H.sparseView(), h);
  EXPECT_TRUE(solver.Solve());
  EXPECT_NEAR(solver.x()[0], -5.0, 1e-10);
}

TEST(OsqpSolverTest, ConstrainedTest) {
  MatXd A = MatXd::Identity(3, 3);
  VecXd b = VecXd::Ones(3) * 5.0;
  MatXd G = MatXd::Zero(1, 3);
  VecXd g = VecXd::Zero(1);
  MatXd H = MatXd::Identity(3, 3);
  VecXd h = VecXd::Ones(3) * 10.0;
  OsqpSolver solver(A.sparseView(), b, G.sparseView(), g, H.sparseView(), h);
  EXPECT_TRUE(solver.Solve());
  EXPECT_NEAR(solver.x()[0], 10.0, 1e-7);
  EXPECT_NEAR(solver.x()[1], 10.0, 1e-7);
  EXPECT_NEAR(solver.x()[2], 10.0, 1e-7);
}

}  // namespace
}  // namespace qcraft
