#include "onboard/planner/router/route.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/file_util.h"

namespace qcraft {
namespace planner {

void Route::FromProto(const RouteProto &route_proto) {
  lane_path_ = std::make_unique<const CompositeLanePath>(semantic_map_manager_,
                                                         route_proto);
  routing_request_ = route_proto.routing_request();
  update_id_ = route_proto.update_id();

  avoid_lanes_.clear();
  for (const auto &avoid_lane_id : route_proto.avoid_lanes()) {
    avoid_lanes_.insert(avoid_lane_id);
  }
  section_seq_ = RouteSectionSequence(
      CompositeLanePath(semantic_map_manager_, route_proto.lane_path()),
      semantic_map_manager_);
  Build();
}

void Route::ToProto(RouteProto *route_proto) const {
  lane_path_->ToProto(route_proto->mutable_lane_path());
  *route_proto->mutable_routing_request() = routing_request_;
  route_proto->set_update_id(update_id_);
  section_seq_.ToProto(route_proto->mutable_route_section_sequence());

  route_proto->mutable_avoid_lanes()->Reserve(avoid_lanes_.size());
  for (const auto &avoid_lane_id : avoid_lanes_) {
    route_proto->add_avoid_lanes(avoid_lane_id);
  }
}

double Route::GetMaxDrivingDistance(mapping::ElementId lane_id) const {
  return FindWithDefault(lane_driving_dis_map_, lane_id, 0.0);
}

double Route::GetMaxDrivingDistance(
    const mapping::LanePoint &lane_point) const {
  const double max_dd_from_end = GetMaxDrivingDistance(lane_point.lane_id());
  const auto &lane_info =
      semantic_map_manager_->FindLaneInfoOrDie(lane_point.lane_id());
  const auto *section = GetFirstOccurrenceOfSecion(lane_info.section_id);
  QCHECK(section);
  return max_dd_from_end +
         lane_info.length() * (section->end_fraction - lane_point.fraction());
}

const RouteSectionSequence::RouteSection *Route::GetFirstOccurrenceOfSecion(
    int64_t section_id) const {
  for (const auto &sec : section_seq_.sections()) {
    if (sec.id == section_id) return &sec;
  }
  return nullptr;
}

// Since lane_driving_dis_map_ is still used in lane change decider,
// we reserve it and delete later.
void Route::Build() {
  lane_driving_dis_map_.clear();
  for (const auto &sec : section_seq_.sections()) {
    for (const auto &lane : sec.lanes) {
      // Record the first occurrence of lane
      if (!ContainsKey(lane_driving_dis_map_, lane.first))
        lane_driving_dis_map_[lane.first] = lane.second;
    }
  }
}

}  // namespace planner
}  // namespace qcraft
