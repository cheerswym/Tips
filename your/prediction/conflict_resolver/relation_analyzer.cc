#include "onboard/prediction/conflict_resolver/relation_analyzer.h"

namespace qcraft {
namespace prediction {

namespace {
double DecideInfluenceThreshold(const ObjectProto& agent_1,
                                const ObjectProto& agent_2) {
  const Box2d bb_box_1(agent_1.bounding_box());
  const Box2d bb_box_2(agent_2.bounding_box());
  return bb_box_1.diagonal() * 0.5 + bb_box_2.diagonal() * 0.5;
}

// pair.first, closest distance b/t segments (within a radius from the point)
// and the point. pair.second, the closest segment idx.
// Search segments within a radius from a position
// (x, y), calculate the closest distance between the segments and the
// point. Return the distance and the closest segment idx as a pair.
absl::StatusOr<std::pair<double, int>> ClosestDistanceWithinRadius(
    const SegmentMatcherKdtree& segment_matcher, const Vec2d& pos,
    double radius) {
  const auto segment_idxes =
      segment_matcher.GetSegmentIndexInRadius(pos.x(), pos.y(), radius);
  if (segment_idxes.empty()) {
    return absl::NotFoundError(absl::StrFormat(
        "Can not find close segments within influence threshold: %f.", radius));
  }
  int closest_idx = -1;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (const auto& idx : segment_idxes) {
    const auto* segment = segment_matcher.GetSegmentByIndex(idx);
    if (segment == nullptr) {
      return absl::InternalError("Querying segment by id fails.");
    }
    const double distance = segment->DistanceTo(pos);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_idx = idx;
    }
  }
  QCHECK_NE(closest_idx, -1);
  return std::make_pair(closest_distance, closest_idx);
}

[[maybe_unused]] absl::StatusOr<double> ClosestDistance(
    const SegmentMatcherKdtree& segment_matcher, const Vec2d& pos) {
  int nearest_segment_idx = -1;
  if (segment_matcher.GetNearestSegmentIndex(pos.x(), pos.y(),
                                             &nearest_segment_idx) == false) {
    return absl::NotFoundError("Cannot find nearest segment index.");
  }
  const auto* nearest_segment =
      segment_matcher.GetSegmentByIndex(nearest_segment_idx);
  if (nearest_segment == nullptr) {
    return absl::InternalError("Querying nearest segment not existing.");
  }
  return nearest_segment->DistanceTo(pos);
}

}  // namespace

AgentRelationType AnalyzeTrajectoryRelation(
    const AgentRelationAnalyzerInput& agent_1,
    const AgentRelationAnalyzerInput& agent_2) {
  const auto& traj_1 = *agent_1.trajectory;
  const auto& traj_2 = *agent_2.trajectory;
  if (traj_1.type() == PT_STATIONARY && traj_2.type() == PT_STATIONARY) {
    return AgentRelationType::ART_NONE;
  }

  if (traj_2.type() == PT_STATIONARY) {
    const auto reversed_relation = AnalyzeTrajectoryRelation(agent_2, agent_1);
    if (reversed_relation == AgentRelationType::ART_NONE) {
      return reversed_relation;
    }
    return reversed_relation == AgentRelationType::ART_PASS
               ? AgentRelationType::ART_YIELD
               : AgentRelationType::ART_PASS;
  }

  const double influence_threshold =
      DecideInfluenceThreshold(*agent_1.object_proto, *agent_2.object_proto);
  if (traj_1.type() == PT_STATIONARY) {
    // If one is stationary, only check if closest distance is within threshold.
    const auto& object_proto = *agent_1.object_proto;
    const Vec2d pos(object_proto.pos());
    const auto& segment_matcher = *agent_2.segments;
    const auto closest_pair_or = ClosestDistanceWithinRadius(
        segment_matcher, pos,
        /*search_radius=*/2.0 * influence_threshold);
    if (!closest_pair_or.ok()) {
      return AgentRelationType::ART_NONE;
    }
    if (closest_pair_or->first < influence_threshold) {
      // Influencing. Object 1 is stationary, always the influencer.
      return AgentRelationType::ART_PASS;
    }
    return AgentRelationType::ART_NONE;
  }

  const PredictedTrajectoryPoint* closest_point_on_traj_1 = nullptr;
  int closest_segment_idx = -1;
  double min_distance = std::numeric_limits<double>::infinity();

  // Use trajectory point on object 1 to check against segment matcher from
  // object 2.
  const auto& segment_matcher = *agent_2.segments;
  for (const auto& point : traj_1.points()) {
    const auto& pos = point.pos();
    // TODO(changqing) : Influence threshold too wide?
    const auto closest_pair_or = ClosestDistanceWithinRadius(
        segment_matcher, pos, /*search_radius=*/2.0 * influence_threshold);
    if (!closest_pair_or.ok()) {
      // Cannot find matching segment within influence radius just for this
      // point. So continue.
      continue;
    }
    const auto& distance = closest_pair_or->first;
    const auto& segment_idx = closest_pair_or->second;
    if (distance < min_distance) {
      min_distance = distance;
      closest_point_on_traj_1 = &point;
      closest_segment_idx = segment_idx;
    }
  }
  if (closest_point_on_traj_1 == nullptr) {
    // Cannot find point on trajectory 1 close enough to trajectory 2.
    return AgentRelationType::ART_NONE;
  }
  // Segments are built based on predicted trajectory points.
  QCHECK_LT(closest_segment_idx, traj_2.points().size());
  // Try query trajectory point directly using segment_idx.
  const auto& closest_point_on_traj_2 = traj_2.points()[closest_segment_idx];

  const auto t1 = closest_point_on_traj_1->t();
  const auto t2 = closest_point_on_traj_2.t();
  if (t1 < t2) {
    return AgentRelationType::ART_PASS;
  }
  return AgentRelationType::ART_YIELD;
}
}  // namespace prediction
}  // namespace qcraft
