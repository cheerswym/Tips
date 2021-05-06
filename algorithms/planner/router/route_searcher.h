#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SEARCHER_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SEARCHER_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/route_section_sequence.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

class RouteSearcher {
 public:
  RouteSearcher(const SemanticMapManager *semantic_map_manager,
                bool use_time = true)
      : semantic_map_manager_(semantic_map_manager), use_time_(use_time) {}

  bool SearchForRoute(mapping::LanePoint origin,
                      absl::Span<const mapping::LanePoint> destinations,
                      RouteProto *route_proto) const;

  bool SearchForRoutePathToLanePoints(
      const mapping::LanePoint &origin,
      absl::Span<const mapping::LanePoint> destinations,
      CompositeLanePath *route_path) const;

  bool SearchForRoutePathWithLaneCosts(
      const mapping::LanePoint &origin,
      absl::Span<const mapping::LanePoint> destinations,
      const absl::flat_hash_map<mapping::ElementId, double> &lane_costs,
      CompositeLanePath *route_path) const;

  bool SearchForRouteSections(
      const mapping::LanePoint &origin,
      absl::Span<const mapping::LanePoint> destinations,
      absl::Span<const double> section_costs,
      std::vector<mapping::ElementId> *section_ids) const;

  void CreateDefaultLaneCosts(
      absl::flat_hash_map<mapping::ElementId, double> *lane_costs) const;

  void CreateDefaultSectionCosts(std::vector<double> *section_costs) const;

 private:
  bool SearchForRouteToDestination(
      mapping::LanePoint origin, mapping::LanePoint destination,
      const absl::flat_hash_map<mapping::ElementId, double> &lane_costs,
      mapping::LanePath *lane_path) const;

  bool SearchForRouteBySections(
      mapping::LanePoint origin, mapping::LanePoint destination,
      absl::Span<const double> section_costs,
      std::vector<mapping::ElementId> *section_ids) const;

  std::vector<std::pair<mapping::LaneNeighborInfo, double>>
  SearchForNeighborsInfo(
      const mapping::LanePoint &start_point,
      const absl::flat_hash_map<mapping::ElementId, mapping::ElementId> &from)
      const;

  CompositeLanePath BuildCompositeLanePathFromLaneIds(
      absl::Span<const mapping::ElementId> lane_ids, double start_fraction,
      double end_fraction) const;
  bool is_use_time() const { return use_time_; }

  const SemanticMapManager *const semantic_map_manager_;
  bool use_time_ = true;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_ROUTE_SEARCHER_H_
