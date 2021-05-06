#include "onboard/planner/decision/precedence_decider.h"

#include <algorithm>

#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

PrecedenceLevel DecidePrecedenceForVehicle(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &av_path,
    const mapping::LanePath &agent_path,
    const prediction::PredictedTrajectory &agent_traj) {
  QCHECK(!agent_traj.points().empty());
  const SmoothLanePath av_smooth_lane_path(psmm.semantic_map_manager(),
                                           av_path);
  const SmoothLanePath agent_smooth_lane_path(psmm.semantic_map_manager(),
                                              agent_path);

  // If the agent is off road, it automatically has lower precedence.
  constexpr double kDistanceThresholdForOffroad = 3.0;  // Center to lane dist.
  if ((Vec2d(agent_smooth_lane_path.Sample(0.0)) -
       agent_traj.points().front().pos())
          .norm() > kDistanceThresholdForOffroad) {
    VLOG(2) << "Precedence for AV because agent is off-road.";
    return PRECEDENCE_AV;
  }

  return DecidePrecedenceForLanePaths(
      psmm, av_path, agent_path, av_smooth_lane_path, agent_smooth_lane_path);
}

PrecedenceLevel DecidePrecedenceForLanePaths(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &av_path,
    const mapping::LanePath &agent_path) {
  const SmoothLanePath av_smooth_lane_path(psmm.semantic_map_manager(),
                                           av_path);
  const SmoothLanePath agent_smooth_lane_path(psmm.semantic_map_manager(),
                                              agent_path);
  return DecidePrecedenceForLanePaths(
      psmm, av_path, agent_path, av_smooth_lane_path, agent_smooth_lane_path);
}

PrecedenceLevel DecidePrecedenceForLanePaths(
    const PlannerSemanticMapManager &psmm, const mapping::LanePath &av_path,
    const mapping::LanePath &agent_path,
    const SmoothLanePath &av_smooth_lane_path,
    const SmoothLanePath &agent_smooth_lane_path) {
  // Check stop signs. The path with stop sign has lower precedence. If both
  // have stop signs, both should be equal in precedence.
  VLOG(2) << "av_path: " << av_path.DebugString();
  VLOG(2) << "agent_path: " << agent_path.DebugString();

  bool av_has_stop_sign = false;
  for (int i = 0; i < av_path.lane_ids().size(); ++i) {
    const mapping::LaneProto &lane_proto =
        psmm.FindLaneByIdOrDie(av_path.lane_id(i));
    if (lane_proto.has_endpoint_associated_stop_sign()) {
      av_has_stop_sign = true;
      break;
    }
  }

  bool agent_has_stop_sign = false;
  for (int i = 0; i < agent_path.lane_ids().size(); ++i) {
    const mapping::LaneProto &lane_proto =
        psmm.FindLaneByIdOrDie(agent_path.lane_id(i));
    if (lane_proto.has_endpoint_associated_stop_sign()) {
      agent_has_stop_sign = true;
      break;
    }
  }

  VLOG(2) << "AV has stop sign: " << av_has_stop_sign;
  VLOG(2) << "Agent has stop sign: " << agent_has_stop_sign;
  if (agent_has_stop_sign && !av_has_stop_sign) {
    VLOG(2) << "AV has precedence due to stop sign in agent path.";
    return PRECEDENCE_AV;
  }
  if (av_has_stop_sign && !agent_has_stop_sign) {
    VLOG(2) << "Agent has precedence due to stop sign in AV path.";
    return PRECEDENCE_AGENT;
  }
  if (agent_has_stop_sign && av_has_stop_sign) {
    VLOG(2) << "AV and agent both have stop signs; precedence is equal.";
    return PRECEDENCE_EQUAL;
  }
  VLOG(2) << "Neither AV nor agent has stop sign.";

  // TODO(Fang) handle lane-change precedence.

  // No stop signs involved. Check speed limit.
  double av_mean_speed_limit = 0.0;
  for (int i = 0; i < av_path.lane_ids().size(); ++i) {
    const mapping::LaneProto &lane_proto =
        psmm.FindLaneByIdOrDie(av_path.lane_id(i));
    const double lane_length = av_smooth_lane_path.lane_length(i);
    av_mean_speed_limit += lane_length * Mph2Mps(lane_proto.speed_limit_mph());
  }
  av_mean_speed_limit /= av_smooth_lane_path.path_length();

  double agent_mean_speed_limit = 0.0;
  for (int i = 0; i < agent_path.lane_ids().size(); ++i) {
    const mapping::LaneProto &lane_proto =
        psmm.FindLaneByIdOrDie(agent_path.lane_id(i));
    const double lane_speed_limit = Mph2Mps(lane_proto.speed_limit_mph());
    if (agent_smooth_lane_path.path_length() == 0.0) {
      agent_mean_speed_limit = lane_speed_limit;
      break;
    }
    const double lane_length = agent_smooth_lane_path.lane_length(i);
    agent_mean_speed_limit +=
        lane_speed_limit * lane_length / agent_smooth_lane_path.path_length();
  }

  VLOG(2) << "AV path mean speed limit: " << av_mean_speed_limit;
  VLOG(2) << "Agent path mean speed limit: " << agent_mean_speed_limit;
  constexpr double kSpeedLimitThreshold = 1.0;  // 2.2mph
  if (av_mean_speed_limit - agent_mean_speed_limit > kSpeedLimitThreshold) {
    VLOG(2) << "AV has precedence due to higher speed limit.";
    return PRECEDENCE_AV;
  } else if (av_mean_speed_limit - agent_mean_speed_limit <
             -kSpeedLimitThreshold) {
    VLOG(2) << "Agent has precedence due to higher speed limit.";
    return PRECEDENCE_AGENT;
  }
  VLOG(2) << "AV and agent has similar speed limit.";

  // Speed limit is equal. Check lane geometry.
  // BANDAID(Fang) tentative rule for lane geoemtry-based precedence: the road
  // with lower curvature is interpreted as the priority road. This is not true
  // in some cases. Example:
  // https://docs.google.com/drawings/d/1aDRhN41eBpZG6zpZ-jKTcqFaP0sL_leAs7hxFE7HzhU/edit?usp=sharing
  constexpr double kCurvatureMeasurementLength = 10.0;  // m.
  const double measurement_length = std::min(
      kCurvatureMeasurementLength,
      std::min(av_smooth_lane_path.s_max(), agent_smooth_lane_path.s_max()));
  const double av_s0 = av_smooth_lane_path.s_max() - measurement_length;
  const double av_s1 = av_smooth_lane_path.s_max();
  const double agent_s0 = agent_smooth_lane_path.s_max() - measurement_length;
  const double agent_s1 = agent_smooth_lane_path.s_max();
  const double av_curvature_integral = NormalizeAngle(
      Vec2d(av_smooth_lane_path.SampleTangent(av_s1)).FastAngle() -
      Vec2d(av_smooth_lane_path.SampleTangent(av_s0)).FastAngle());
  const double agent_curvature_integral = NormalizeAngle(
      Vec2d(agent_smooth_lane_path.SampleTangent(agent_s1)).FastAngle() -
      Vec2d(agent_smooth_lane_path.SampleTangent(agent_s0)).FastAngle());
  VLOG(2) << "Curvature measurement length: " << measurement_length;
  VLOG(2) << "AV curvature integral: " << av_curvature_integral;
  VLOG(2) << "Agent curvature integral: " << agent_curvature_integral;
  constexpr double kCurvatureThreshold = 0.1;
  if (std::abs(av_curvature_integral) - std::abs(agent_curvature_integral) >
      kCurvatureThreshold) {
    VLOG(2) << "Agent has precedence due to smaller lane curvature.";
    return PRECEDENCE_AGENT;
  }
  if (std::abs(av_curvature_integral) - std::abs(agent_curvature_integral) <
      -kCurvatureThreshold) {
    VLOG(2) << "AV has precedence due to smaller lane curvature.";
    return PRECEDENCE_AV;
  } else {
    VLOG(2)
        << "AV and agent have equal precedence due to similar lane curvature.";
    return PRECEDENCE_EQUAL;
  }
}

}  // namespace planner
}  // namespace qcraft
