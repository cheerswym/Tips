#ifndef ONBOARD_PLANNER_COMMON_PLOT_UTIL_H_
#define ONBOARD_PLANNER_COMMON_PLOT_UTIL_H_

#include <string>

#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

void SendPointsToCanvas(absl::Span<const Vec2d> points,
                        const std::string &channel, vis::Color color);

void SendApolloTrajectoryPointsToCanvas(
    absl::Span<const ApolloTrajectoryPointProto> traj_pts,
    const std::string &channel, vis::Color color);

void SendTrajectoryPointsToCanvas(
    const ::google::protobuf::RepeatedPtrField< ::qcraft::TrajectoryPointProto>
        &traj_pts,
    const std::string &channel, vis::Color color);

void DrawPathSlBoundaryToCanvas(const PathSlBoundary &path_bound,
                                const std::string &channel);

void DrawLanePathToCanvas(const SemanticMapManager &semantic_map_manager,
                          const mapping::LanePath &lp,
                          const std::string &channel, vis::Color color);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_PLOT_UTIL_H_
