#ifndef ONBOARD_PLANNER_COMMON_LANE_PATH_INFO_H_
#define ONBOARD_PLANNER_COMMON_LANE_PATH_INFO_H_

#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"

namespace qcraft::planner {

class LanePathInfo {
 public:
  LanePathInfo() {}
  LanePathInfo(mapping::LanePath lane_path, double len_along_route,
               double path_cost, const SemanticMapManager& smm);

  bool empty() const { return lane_path_.IsEmpty(); }
  const mapping::LanePath& lane_path() const { return lane_path_; }
  mapping::ElementId start_lane_id() const {
    return lane_path_.front().lane_id();
  }

  // If a routed lane change must be executed on this lane path, length
  // decreases by a default lane change distance.
  //
  // [— length_along_route —]
  // ----------------------------------------->
  //
  // [————— length_along_route —————]
  // ----------------------------------------->
  //
  // ------------------------------------------
  //                                          |
  //                                          |
  //                                      destination
  double length_along_route() const { return length_along_route_; }
  double path_cost() const { return path_cost_; }

  FrenetCoordinate ProjectionSL(Vec2d xy) const;
  FrenetCoordinate ProjectionSLInRange(Vec2d xy, double start_s,
                                       double end_s) const;
  Vec2d ProjectionXY(FrenetCoordinate sl) const;

 private:
  mapping::LanePath lane_path_;
  double length_along_route_ = 0.0;
  double path_cost_ = DBL_MAX;

  // Projection system
  std::vector<Vec2d> anchor_points_;
  std::vector<double> anchor_s_;
  std::vector<Vec2d> tangents_;
  std::vector<double> segment_len_inv_;

  // TODO(weijun): potentialy add lane path boundary here.
  // speed_limit
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_COMMON_LANE_PATH_INFO_H_
