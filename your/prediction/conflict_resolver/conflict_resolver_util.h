#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_UTIL_H_
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_UTIL_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/conflict_resolver/object_svt_sample.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
std::pair<planner::DiscretizedPath, planner::SpeedVector>
PredictedTrajectoryToPurePathAndSpeedVector(
    const PredictedTrajectory& trajectory);

inline Vec2d PathPointToVec2d(const PathPoint& pt) {
  return Vec2d(pt.x(), pt.y());
}

ConflictResolverDebugProto::SimpleSpeedProfile
EdgeConnectionToSimpleSpeedProfile(
    absl::Span<const std::string> cost_names, const std::vector<SvtEdge>& edges,
    const SvtEdgeVector<SvtEdgeCost>& search_costs,
    const std::vector<SvtEdgeIndex>& edge_idxes);

ConflictResolverDebugProto::SpeedProfile SpeedVectorToSpeedProfile(
    const planner::SpeedVector& speed_vector,
    const std::vector<SvtEdgeIndex>& edge_idxes);

std::string PrintOriginalAndModifiedPredictedTrajectory(
    const PredictedTrajectory& origin, const PredictedTrajectory& modified);

planner::SpeedPoint SvtStateToSpeedPoint(const SvtState& state, double a);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_CONFLICT_RESOLVER_UTIL_H_
