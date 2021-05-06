#include "onboard/planner/initializer/motion_search_util.h"

#include <algorithm>
#include <memory>
#include <utility>

#include "onboard/math/util.h"

namespace qcraft::planner {
std::vector<double> ConvertStoplineToStopS(
    absl::Span<const ConstraintProto::StopLineProto> stoplines,
    double sdc_length) {
  std::vector<double> stop_s;
  stop_s.reserve(stoplines.size());
  for (const auto &stop_line : stoplines) {
    // Do not consider the end of path boundary stopline here
    if (stop_line.source().has_end_of_path_boundary()) {
      continue;
    }
    stop_s.push_back(std::max(stop_line.s() - sdc_length, 0.0));
  }
  return stop_s;
}

ApolloTrajectoryPointProto MotionState2TrajPoint(
    const MotionState &motion_state, double s, double current_t) {
  PathPoint point;
  point.set_x(motion_state.xy.x());
  point.set_y(motion_state.xy.y());
  point.set_z(0);
  point.set_theta(motion_state.h);
  point.set_kappa(motion_state.k);
  point.set_s(s);

  ApolloTrajectoryPointProto traj_point;
  *(traj_point.mutable_path_point()) = point;
  traj_point.set_v(motion_state.v);
  traj_point.set_a(motion_state.a);
  traj_point.set_relative_time(current_t);

  return traj_point;
}

std::vector<ApolloTrajectoryPointProto> ResampleTrajectoryPoints(
    const std::vector<const MotionForm *> &motions) {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  traj_points.reserve(kTrajResampleNum);

  double s = 0.0;
  auto motion_it = motions.begin();
  double current_motion_start_time = 0,
         current_motion_end_time = (*motion_it)->duration();
  traj_points.emplace_back(MotionState2TrajPoint(
      (*motion_it)->GetStartMotionState(), /*s=*/0, /*current_t=*/0));
  for (int i = 1; i < kTrajResampleNum; ++i) {
    const double current_t = i * kTrajectoryTimeStep;
    if (current_t > current_motion_end_time) {
      ++motion_it;
      // If we reach the end of the trajectory.
      if (motion_it == motions.end()) {
        break;
      }
      current_motion_start_time = current_motion_end_time;
      current_motion_end_time += (*motion_it)->duration();
    }
    const auto motion_state =
        (*motion_it)->State(current_t - current_motion_start_time);

    s += motion_state.xy.DistanceTo(Vec2d(traj_points[i - 1].path_point().x(),
                                          traj_points[i - 1].path_point().y()));
    traj_points.emplace_back(MotionState2TrajPoint(motion_state, s, current_t));
  }

  return traj_points;
}

ApolloTrajectoryPointProto InterpolateTrajectoryPoint(
    const ApolloTrajectoryPointProto obj0,
    const ApolloTrajectoryPointProto obj1, const double t) {
  const double t0 = obj0.relative_time();
  const double t1 = obj1.relative_time();
  const double weight = (t - t0) / (t1 - t0 + kEpsilon);
  PathPoint point;
  const auto &p0 = obj0.path_point();
  const auto &p1 = obj1.path_point();
  point.set_x(Lerp(p0.x(), p1.x(), weight));
  point.set_y(Lerp(p0.y(), p1.y(), weight));
  point.set_z(0);
  point.set_theta(NormalizeAngle(LerpAngle(p0.theta(), p1.theta(), weight)));
  point.set_kappa(Lerp(p0.kappa(), p1.kappa(), weight));
  point.set_s(Lerp(p0.s(), p1.s(), weight));

  ApolloTrajectoryPointProto traj_point;
  *(traj_point.mutable_path_point()) = point;
  traj_point.set_v(Lerp(obj0.v(), obj1.v(), weight));
  traj_point.set_a(Lerp(obj0.a(), obj1.a(), weight));
  traj_point.set_relative_time(t);
  return traj_point;
}

std::vector<ApolloTrajectoryPointProto> ConstructStationaryTraj(
    const MotionState &sdc_motion) {
  const MotionState state{.xy = sdc_motion.xy,
                          .h = sdc_motion.h,
                          .k = sdc_motion.k,
                          .v = 0.0,
                          .a = 0.0};

  std::vector<ApolloTrajectoryPointProto> stationary_traj;
  stationary_traj.reserve(kTrajResampleNum);
  for (int i = 0; i < kTrajResampleNum; ++i) {
    const double current_t = i * kTrajectoryTimeStep;
    stationary_traj.emplace_back(
        MotionState2TrajPoint(state, /*s=*/0.0, current_t));
  }

  return stationary_traj;
}

std::vector<ApolloTrajectoryPointProto> ConstructTrajFromLastEdge(
    const MotionGraph &motion_graph, MotionEdgeIndex last_edge_index) {
  std::vector<const MotionForm *> motions;
  QCHECK_GE(last_edge_index.value(), 0);
  while (last_edge_index != MotionEdgeVector<MotionEdge>::kInvalidIndex) {
    const auto &motion_edge = motion_graph.GetMotionEdge(last_edge_index);
    motions.emplace_back(motion_edge.motion);
    last_edge_index = motion_edge.prev_edge;
  }
  std::reverse(motions.begin(), motions.end());
  return ResampleTrajectoryPoints(motions);
}

// TODO(changqing): modify this function with cache.
// MotionEdgeIndex CreateStationaryMotion(const MotionState &sdc_motion,
//                                        MotionNodeIndex sdc_node_index,
//                                        const GeometryNodeIndex
//                                        &sdc_geom_node, MotionGraph
//                                        *mutable_motion_graph) {
//   std::unique_ptr<MotionForm> stationary_motion =
//       std::make_unique<StationaryMotion>(
//           kTrajectoryTimeHorizon,
//           GeometryState{.xy = sdc_motion.xy,
//                         .h = sdc_motion.h,
//                         .k = sdc_motion.k,
//                         .accumulated_s = sdc_motion.accumulated_s,
//                         .l = sdc_motion.l});
//   auto set2zero_sdc_motion = sdc_motion;
//   set2zero_sdc_motion.v = 0.0;
//   const auto end_node_index =
//       mutable_motion_graph->AddMotionNode(set2zero_sdc_motion,
//       sdc_geom_node);
//   return mutable_motion_graph->AddMotionEdge(
//       sdc_node_index, end_node_index, std::move(stationary_motion),
//       sdc_geom_node, MotionEdgeVector<MotionEdge>::kInvalidIndex);
// }
}  // namespace qcraft::planner
