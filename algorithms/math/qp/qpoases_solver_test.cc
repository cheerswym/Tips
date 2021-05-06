#include "onboard/math/qp/qpoases_solver.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(QpoasesSolverTest, SimpleTest) {
  MatXd A = MatXd::Ones(1, 1);
  VecXd b = VecXd::Ones(1) * 5.0;
  MatXd G = MatXd::Zero(0, 1);
  VecXd g = VecXd::Zero(0);
  MatXd H = MatXd::Zero(0, 1);
  VecXd h = VecXd::Zero(0);
  QpoasesSolver solver(A, b, G, g, H, h);
  EXPECT_TRUE(solver.Solve());
  EXPECT_NEAR(solver.x()[0], -5.0, 1e-10);
}

TEST(QpoasesSolverTest, ConstrainedTest) {
  MatXd A = MatXd::Identity(3, 3);
  VecXd b = VecXd::Ones(3) * 5.0;
  MatXd G = MatXd::Zero(0, 3);
  VecXd g = VecXd::Zero(0);
  MatXd H = MatXd::Identity(3, 3);
  VecXd h = VecXd::Ones(3) * 10.0;
  QpoasesSolver solver(A, b, G, g, H, h);
  EXPECT_TRUE(solver.Solve());
  EXPECT_NEAR(solver.x()[0], 10.0, 1e-10);
  EXPECT_NEAR(solver.x()[1], 10.0, 1e-10);
  EXPECT_NEAR(solver.x()[2], 10.0, 1e-10);
}

}  // namespace
}  // namespace qcraft
