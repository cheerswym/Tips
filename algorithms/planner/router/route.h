#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_H_

#include <memory>
#include <queue>
#include <string>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "gflags/gflags.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_section_sequence.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

struct BusStationStopArea {
  explicit BusStationStopArea(const SemanticMapManager *semantic_map_manager)
      : stop_lane_path(semantic_map_manager) {}
  std::string bus_station_name;
  mapping::LanePoint real_stop_lane_point;
  mapping::LanePath stop_lane_path;
  Polygon2d polygon;
};

class Route {
 public:
  explicit Route(const SemanticMapManager *semantic_map_manager,
                 const RouteProto &route_proto)
      : semantic_map_manager_(semantic_map_manager) {
    FromProto(route_proto);
  }

  explicit Route(const SemanticMapManager *semantic_map_manager,
                 std::unique_ptr<CompositeLanePath> lane_path,
                 RouteSectionSequence section_seq,
                 RoutingRequestProto routing_request, int64_t update_id,
                 absl::flat_hash_set<mapping::ElementId> avoid_lanes)
      : lane_path_(std::move(lane_path)),
        section_seq_(std::move(section_seq)),
        routing_request_(std::move(routing_request)),
        update_id_(update_id),
        semantic_map_manager_(semantic_map_manager),
        avoid_lanes_(std::move(avoid_lanes)) {
    Build();
  }

  const CompositeLanePath &lane_path() const {
    QCHECK(lane_path_ != nullptr);
    return *lane_path_;
  }

  const RouteSectionSequence &section_sequence() const { return section_seq_; }

  // Max driving distance on route from lane's end
  double GetMaxDrivingDistance(mapping::ElementId lane_id) const;
  // Max driving distance on route from lane point
  double GetMaxDrivingDistance(const mapping::LanePoint &lane_point) const;

  const RouteSectionSequence::RouteSection *GetFirstOccurrenceOfSecion(
      int64_t section_id) const;

  int64_t update_id() const { return update_id_; }
  void set_update_id(int64_t update_id) { update_id_ = update_id; }

  const RoutingRequestProto &routing_request() const {
    return routing_request_;
  }

  void set_routing_request(const RoutingRequestProto &routing_request) {
    routing_request_ = routing_request;
  }

  const absl::flat_hash_set<mapping::ElementId> &avoid_lanes() const {
    return avoid_lanes_;
  }

  absl::flat_hash_set<mapping::ElementId> *mutable_avoid_lanes() {
    return &avoid_lanes_;
  }

  void set_distance(double distance) { distance_ = distance; }
  void set_eta(double eta_secs) { eta_ = eta_secs; }

  void FromProto(const RouteProto &route_proto);

  void ToProto(RouteProto *route_proto) const;

 private:
  void Build();

 private:
  std::unique_ptr<const CompositeLanePath> lane_path_;
  RouteSectionSequence section_seq_;

  absl::flat_hash_map<mapping::ElementId, double>
      lane_driving_dis_map_;  // Delete

  RoutingRequestProto routing_request_;
  int64_t update_id_ = 0;

  const SemanticMapManager *semantic_map_manager_;
  absl::flat_hash_set<mapping::ElementId> avoid_lanes_;

  double distance_ = 0.0;  // The total distance;
  double eta_ = 0.0;       // The estimate time of arrival in secs;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_H_
