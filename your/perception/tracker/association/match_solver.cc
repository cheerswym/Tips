#include "onboard/perception/tracker/association/match_solver.h"

#include <limits>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"

namespace qcraft::tracker {
namespace association {

bool MatchSolver::Init(const association::MatchSolverProto& config_proto) {
  if (!config_proto.has_solver_algorithm()) return false;
  config_ = config_proto;
  return true;
}

std::vector<int> MatchSolver::Solve(
    const Eigen::MatrixXd& similarity_matrix) const {
  SCOPED_QTRACE("MatchSolver::Solve");

  if (!config_.has_value()) {
    QLOG(FATAL) << "MatchSolverIsNotInit.";
  }

  switch (config_->solver_algorithm()) {
    case association::MatchSolverProto::GNN:
      return GreedyNearestNeighbor(similarity_matrix);
    case association::MatchSolverProto::KUHN_MUNKRES:
      return KuhnMunkres(similarity_matrix);
    case association::MatchSolverProto::GNN_MV1:
      return GreedyNearestNeighborMv1(similarity_matrix);
    default:
      QLOG(FATAL) << "Unknown solver type.";
  }
}

std::vector<int> MatchSolver::KuhnMunkres(
    const Eigen::MatrixXd& weight_matrix) const {
  QLOG(FATAL) << "Not implemented yet.";
  return {};
}

std::vector<int> MatchSolver::GreedyNearestNeighbor(
    const Eigen::MatrixXd& weight_matrix) const {
  SCOPED_QTRACE("MatchSolver::GreedyNearestNeighbor");

  if (!config_.has_value()) {
    QLOG(FATAL) << "MatchSolverIsNotInit.";
  }

  const int rows = weight_matrix.rows();
  const int cols = weight_matrix.cols();
  std::vector<bool> matched_rows(rows);  // measurements
  std::vector<bool> matched_cols(cols);  // tracks
  std::vector<int> match_result(rows, -1);
  while (true) {
    double max_val = -std::numeric_limits<double>::max();
    int max_val_col, max_val_row;
    for (int i = 0; i < cols; ++i) {
      if (matched_cols[i]) continue;
      for (int j = 0; j < rows; ++j) {
        if (matched_rows[j]) continue;
        if (weight_matrix(j, i) > max_val) {
          max_val = weight_matrix(j, i);
          max_val_col = i;
          max_val_row = j;
        }
      }
    }
    if (max_val > 0.0) {
      matched_cols[max_val_col] = true;
      matched_rows[max_val_row] = true;
      match_result[max_val_row] = max_val_col;
    } else {
      break;
    }
  }
  return match_result;
}

std::vector<int> MatchSolver::GreedyNearestNeighborMv1(
    const Eigen::MatrixXd& weight_matrix) const {
  SCOPED_QTRACE("MatchSolver::GreedyNearestNeighborMv1");

  if (!config_.has_value()) {
    QLOG(FATAL) << "MatchSolverIsNotInit.";
  }

  const int rows = weight_matrix.rows();
  const int cols = weight_matrix.cols();
  std::vector<int> match_result(rows, -1);
  for (int r = 0; r < rows; ++r) {
    double max_val = -1.0;
    for (int c = 0; c < cols; ++c) {
      if (weight_matrix(r, c) > 0.0 && weight_matrix(r, c) > max_val) {
        max_val = weight_matrix(r, c);
        match_result[r] = c;
      }
    }
  }
  return match_result;
}

}  // namespace association
}  // namespace qcraft::tracker
