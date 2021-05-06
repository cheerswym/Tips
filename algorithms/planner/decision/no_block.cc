#include "onboard/planner/decision/no_block.h"

#include <limits>
#include <string>
#include <utility>

#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {
std::vector<ConstraintProto::SpeedRegionProto> BuildNoBlockConstraints(
    const PlannerSemanticMapManager &planner_semantic_map_manager,
    const DrivePassage &passage) {
  const auto &lane_path = passage.lane_path();
  absl::flat_hash_map<std::string,
                      std::vector<ConstraintProto::SpeedRegionProto>>
      id_element_map;
  for (const auto &seg : lane_path) {
    const auto &lane_info =
        planner_semantic_map_manager.FindLaneInfoOrDie(seg.lane_id);
    for (const auto &intersection : lane_info.intersections) {
      // Start point.
      const double start_point_s = lane_path.LaneIndexPointToArclength(
          /*lane_index_point=*/{seg.lane_index, intersection.second.x()});
      const auto start_point =
          lane_info.LerpPointFromFraction(intersection.second.x());
      // End point.
      const double end_point_s = lane_path.LaneIndexPointToArclength(
          /*lane_index_point=*/{seg.lane_index, intersection.second.y()});
      const auto end_point =
          lane_info.LerpPointFromFraction(intersection.second.y());
      const std::string constraint_id =
          absl::StrFormat("no_block_%d", intersection.first);

      ConstraintProto::SpeedRegionProto no_block_constraint;
      start_point.ToProto(no_block_constraint.mutable_start_point());
      end_point.ToProto(no_block_constraint.mutable_end_point());
      no_block_constraint.set_start_s(start_point_s +
                                      passage.lane_path_start_s());
      no_block_constraint.set_end_s(end_point_s + passage.lane_path_start_s());
      no_block_constraint.set_max_speed(
          planner_semantic_map_manager.QueryLaneSpeedLimitById(seg.lane_id));
      constexpr double kIntersectionMinSpeed = 0.5;  // m/s
      no_block_constraint.set_min_speed(kIntersectionMinSpeed);
      no_block_constraint.mutable_source()->mutable_no_block()->set_id(
          intersection.first);
      no_block_constraint.set_id(constraint_id);
      id_element_map[constraint_id].push_back(std::move(no_block_constraint));
    }
  }
  std::vector<ConstraintProto::SpeedRegionProto> no_block_constraints;
  no_block_constraints.reserve(id_element_map.size());
  for (const auto &id_element : id_element_map) {
    no_block_constraints.push_back(MergeSameElement(id_element.second));
  }
  return no_block_constraints;
}

}  // namespace planner
}  // namespace qcraft
