#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_

#include <vector>

#include "absl/types/span.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/planner_defs.h"
namespace qcraft::planner {

constexpr double kEpsilon = 1e-6;

std::vector<double> ConvertStoplineToStopS(
    absl::Span<const ConstraintProto::StopLineProto> stoplines,
    double sdc_length);

ApolloTrajectoryPointProto MotionState2TrajPoint(
    const MotionState &motion_state, double s, double current_t);

std::vector<ApolloTrajectoryPointProto> ResampleTrajectoryPoints(
    const std::vector<const MotionForm *> &motions);

ApolloTrajectoryPointProto InterpolateTrajectoryPoint(
    const ApolloTrajectoryPointProto p0, const ApolloTrajectoryPointProto p1,
    const double t);

std::vector<ApolloTrajectoryPointProto> ConstructStationaryTraj(
    const MotionState &sdc_motion);

std::vector<ApolloTrajectoryPointProto> ConstructTrajFromLastEdge(
    const MotionGraph &motion_graph, MotionEdgeIndex last_edge_index);
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_SEARCH_UTIL_H_
