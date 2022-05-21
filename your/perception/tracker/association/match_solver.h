#ifndef ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_MATCH_SOLVER_H_
#define ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_MATCH_SOLVER_H_

#include <optional>
#include <vector>

#include "Eigen/Core"
#include "onboard/perception/tracker/association/proto/associator_config.pb.h"

namespace qcraft::tracker {
namespace association {
class MatchSolver {
 public:
  MatchSolver() = default;

  bool Init(const association::MatchSolverProto& config_proto);

  /* Input similarity matrix:
     Each row is a measurement;
     Each col is a track;
     Output track correspond measurement.
     Result[r] = c, r is measurement, c is track.
     Result[r] = -1 when no match is found.
  */
  std::vector<int> Solve(const Eigen::MatrixXd& similarity_matrix) const;

 private:
  /* Input similarity matrix:
     Each row is a measurement;
     Each col is a track;
     Output track correspond measurement.
     Result[r] = c, r is measurement, c is track.
     Result[r] = -1 when no match is found.
  */
  std::vector<int> GreedyNearestNeighbor(
      const Eigen::MatrixXd& similarity_matrix) const;

  /* Input similarity matrix:
     Each row is a measurement;
     Each col is a track;
     Output track correspond measurement.
     Result[r] = c, r is measurement, c is track.
     Result[r] = -1 when no match is found.
  */
  std::vector<int> KuhnMunkres(const Eigen::MatrixXd& weight_matrix) const;

  /* Input similarity matrix:
     Each row is a measurement;
     Each col is a track;
     Output track correspond measurement.
     Result[r] = c, r is measurement, c is track.
     Result[r] = -1 when no match is found.
  */
  std::vector<int> GreedyNearestNeighborMv1(
      const Eigen::MatrixXd& similarity_matrix) const;

  std::optional<association::MatchSolverProto> config_ = std::nullopt;
};

}  // namespace association
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_ASSOCIATION_MATCH_SOLVER_H_
