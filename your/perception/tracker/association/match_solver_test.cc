#include "onboard/perception/tracker/association/match_solver.h"

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft::tracker {
namespace association {

TEST(MatchSolver, TestGreedyNearestNeighbor) {
  /* Weight matrix.
     1.0   3.0   5.0  (10.1) 2.0
    (10.2) 7.0   6.0   2.0   1.0
     1.2  (10.3) 4.5   3.5   2.2
     2.1   5.1  (10.6) 4.3   1.3
     1.2   4.2   6.3   2.7  (10.9)
  */
  Eigen::MatrixXd weight_matrix;
  weight_matrix.resize(5, 5);
  weight_matrix << 1.0, 3.0, 5.0, 10.1, 2.0, 10.2, 7.0, 6.0, 2.0, 1.0, 1.2,
      10.3, 4.5, 3.5, 2.2, 2.1, 5.1, 10.6, 4.3, 1.3, 1.2, 4.2, 6.3, 2.7, 10.9;

  std::vector<int> ground_truth = {3, 0, 1, 2, 4};
  // Solve
  MatchSolver match_solver;
  association::MatchSolverProto config;
  config.set_solver_algorithm(association::MatchSolverProto::GNN);
  EXPECT_TRUE(match_solver.Init(config));
  const auto match_result = match_solver.Solve(weight_matrix);

  // Check result.
  ASSERT_EQ(ground_truth.size(), match_result.size());
  for (size_t i = 0; i < ground_truth.size(); ++i) {
    EXPECT_EQ(ground_truth[i], match_result[i]);
  }
}

TEST(MatchSolver, TestGreedyNearestNeighborMv1) {
  /* Weight matrix.
     1.0   3.0   5.0  (10.1) 2.0
    (10.2) 7.0   6.0   2.0   1.0
    (10.3) 1.2   4.5   3.5   2.2
     2.1   5.1  (10.6) 4.3   1.3
     1.2   4.2   6.3   2.7  (10.9)
  */
  Eigen::MatrixXd weight_matrix;
  weight_matrix.resize(5, 5);
  weight_matrix << 1.0, 3.0, 5.0, 10.1, 2.0, 10.2, 7.0, 6.0, 2.0, 1.0, 10.3,
      1.2, 4.5, 3.5, 2.2, 2.1, 5.1, 10.6, 4.3, 1.3, 1.2, 4.2, 6.3, 2.7, 10.9;

  std::vector<int> ground_truth = {3, 0, 0, 2, 4};
  // Solve
  MatchSolver match_solver;
  association::MatchSolverProto config;
  config.set_solver_algorithm(association::MatchSolverProto::GNN_MV1);
  EXPECT_TRUE(match_solver.Init(config));
  const auto match_result = match_solver.Solve(weight_matrix);

  // Check result.
  ASSERT_EQ(ground_truth.size(), match_result.size());
  for (size_t i = 0; i < ground_truth.size(); ++i) {
    EXPECT_EQ(ground_truth[i], match_result[i]);
  }
}

}  // namespace association
}  // namespace qcraft::tracker

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
