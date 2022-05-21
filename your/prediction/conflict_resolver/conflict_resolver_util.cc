#include "onboard/prediction/conflict_resolver/conflict_resolver_util.h"

#include "absl/strings/str_format.h"
namespace qcraft {
namespace prediction {

namespace {

bool AreCumulativeDistanceClose(const PathPoint& p1, const PathPoint& p2) {
  constexpr double kCloseThreshold = 1e-6;
  return std::fabs(p1.s() - p2.s()) < kCloseThreshold;
}

PathPoint PredictedPointToPathPoint(const PredictedTrajectoryPoint& pt) {
  PathPoint path_pt;
  path_pt.set_x(pt.pos().x());
  path_pt.set_y(pt.pos().y());
  path_pt.set_s(pt.s());
  path_pt.set_theta(pt.theta());
  path_pt.set_kappa(pt.kappa());
  return path_pt;
}

planner::SpeedPoint PredictedPointToSpeedPoint(
    const PredictedTrajectoryPoint& pt) {
  // Cant calculate jerk for now.
  return planner::SpeedPoint(pt.t(), pt.s(), pt.v(), pt.a(), /*j=*/0.0);
}

}  // namespace

// Resolve predicted trajectories that might have stopped and thus fail to
// provide valid segments to build segment matcher.
std::pair<planner::DiscretizedPath, planner::SpeedVector>
PredictedTrajectoryToPurePathAndSpeedVector(
    const PredictedTrajectory& trajectory) {
  const auto& predicted_points = trajectory.points();
  planner::DiscretizedPath path;
  std::vector<planner::SpeedPoint> speed_points;
  speed_points.reserve(predicted_points.size());
  path.reserve(predicted_points.size());
  path.push_back(PredictedPointToPathPoint(predicted_points.front()));
  speed_points.push_back(PredictedPointToSpeedPoint(predicted_points.front()));

  for (int i = 1; i < predicted_points.size(); ++i) {
    const auto& predicted_point = predicted_points[i];
    const auto path_pt = PredictedPointToPathPoint(predicted_point);
    if (AreCumulativeDistanceClose(path_pt, path.back())) {
      // Do not add this point if it is close to the previous added point.
      continue;
    }
    path.push_back(path_pt);
    speed_points.push_back(PredictedPointToSpeedPoint(predicted_point));
  }
  return std::make_pair(std::move(path),
                        planner::SpeedVector(std::move(speed_points)));
}

ConflictResolverDebugProto::SimpleSpeedProfile
EdgeConnectionToSimpleSpeedProfile(
    absl::Span<const std::string> cost_names, const std::vector<SvtEdge>& edges,
    const SvtEdgeVector<SvtEdgeCost>& search_costs,
    const std::vector<SvtEdgeIndex>& edge_idxes) {
  ConflictResolverDebugProto::SimpleSpeedProfile profile;
  const auto& final_edge = edges[edge_idxes.back().value()];
  if (final_edge.edge_cost != nullptr) {
    const auto& sum_cost = final_edge.edge_cost->sum_cost;
    profile.set_total_cost(sum_cost);
    profile.set_eval_cost(sum_cost / final_edge.final_t);
  }
  profile.set_final_t(final_edge.final_t);
  profile.set_final_v(final_edge.final_v);
  if (final_edge.states != nullptr && !final_edge.states->empty()) {
    profile.set_final_s(final_edge.states->back().s);
  }
  for (const auto& edge_idx : edge_idxes) {
    *profile.add_edges() =
        edges[edge_idx.value()].ToSimpleSpeedProfileProto(cost_names);
  }
  return profile;
}

ConflictResolverDebugProto::SpeedProfile SpeedVectorToSpeedProfile(
    const planner::SpeedVector& speed_vector,
    const std::vector<SvtEdgeIndex>& edge_idxes) {
  ConflictResolverDebugProto::SpeedProfile speed_profile;
  speed_profile.mutable_points()->Reserve(speed_vector.size());
  for (const auto& point : speed_vector) {
    ConflictResolverDebugProto::SpeedProfile::SpeedPoint pt;
    pt.set_s(point.s());
    pt.set_v(point.v());
    pt.set_t(point.t());
    pt.set_a(point.a());
    *speed_profile.add_points() = std::move(pt);
  }
  for (const auto& idx : edge_idxes) {
    speed_profile.add_edge_idxes(idx.value());
  }
  return speed_profile;
}

std::string PrintOriginalAndModifiedPredictedTrajectory(
    const PredictedTrajectory& origin, const PredictedTrajectory& modified) {
  std::vector<std::string> results;
  const auto& origin_points = origin.points();
  const auto& modified_points = modified.points();
  results.reserve(std::max(origin_points.size(), modified_points.size()));

  auto origin_it = origin_points.begin();
  auto modified_it = modified_points.begin();

  while (origin_it != origin_points.end() ||
         modified_it != modified_points.end()) {
    std::string compare_string;
    if (origin_it != origin_points.end()) {
      compare_string +=
          absl::StrFormat("Origin: s: %f, v: %f, a: %f, t: %f.", origin_it->s(),
                          origin_it->v(), origin_it->a(), origin_it->t());
    }
    if (modified_it != modified_points.end()) {
      compare_string += absl::StrFormat("Modified: s: %f, v: %f, a: %f, t: %f.",
                                        modified_it->s(), modified_it->v(),
                                        modified_it->a(), modified_it->t());
    }
    results.push_back(std::move(compare_string));
    if (origin_it != origin_points.end()) origin_it++;
    if (modified_it != modified_points.end()) modified_it++;
  }
  return absl::StrJoin(results, "\n");
}

planner::SpeedPoint SvtStateToSpeedPoint(const SvtState& state, double a) {
  return planner::SpeedPoint(state.t, state.s, state.v, a, 0.0);
}
}  // namespace prediction
}  // namespace qcraft
