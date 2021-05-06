#include "onboard/planner/toll_util.h"

#include "onboard/maps/semantic_map_manager.h"

namespace qcraft {
namespace planner {

std::vector<std::pair<CompositeLanePath::LaneIndexPoint, double>>
FindTollsAlongLanePath(const SemanticMapManager &semantic_map_manager,
                       const CompositeLanePath &lane_path,
                       const std::vector<double> &additional_tolls) {
  std::vector<std::pair<CompositeLanePath::LaneIndexPoint, double>> tolls;
  for (auto lane_it = lane_path.begin(); lane_it != lane_path.end();
       ++lane_it) {
    const mapping::ElementId lane_id = (*lane_it).lane_id;
    const auto *lane_proto_ptr =
        semantic_map_manager.FindLaneByIdOrNull(lane_id);
    if (lane_proto_ptr == nullptr) {
      continue;
    }
    const mapping::LaneProto &lane_proto = *lane_proto_ptr;
    if (lane_proto.endpoint_toll()) {
      const CompositeLanePath::LaneIndexPoint lip(lane_it.composite_index(),
                                                  1.0);
      tolls.emplace_back(lip, lane_path.LaneIndexPointToArclength(lip));
    }
  }

  for (const double additional_toll : additional_tolls) {
    if (std::isinf(additional_toll)) continue;
    tolls.emplace_back(lane_path.ArclengthToLaneIndexPoint(additional_toll),
                       additional_toll);
  }

  return tolls;
}

}  // namespace planner
}  // namespace qcraft
