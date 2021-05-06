#ifndef ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_A_STAR_HEURISTICS_H
#define ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_A_STAR_HEURISTICS_H

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/path_manager_util.h"

namespace qcraft {
namespace planner {

class AStarHeuristics {
 public:
  AStarHeuristics(const AABox2d &region, double grid_resolution,
                  double vehicle_radius, double end_x, double end_y)
      : region_(region),
        grid_resolution_(grid_resolution),
        vehicle_radius_(vehicle_radius) {
    end_grid_x_ = FloorToInt((end_x - region.min_x()) / grid_resolution_);
    end_grid_y_ = FloorToInt((end_y - region.min_y()) / grid_resolution_);
    const int row = FloorToInt(region.length() / grid_resolution_);
    const int column = FloorToInt(region.width() / grid_resolution_);
    std::vector<std::vector<double>> cost_map(
        row,
        std::vector<double>(column, std::numeric_limits<double>::infinity()));
    cost_map_ = std::move(cost_map);
  }
  ~AStarHeuristics() = default;

  bool GenerateCostMap(
      const std::vector<FreespaceBoundary> &boundaries,
      const std::vector<std::pair<std::string, Polygon2d>> &stationary_objects);
  double GetHeuristicsCost(double x, double y) const;

 private:
  AABox2d region_;
  double grid_resolution_;
  double vehicle_radius_;
  int end_grid_x_;
  int end_grid_y_;
  std::vector<std::vector<double>> cost_map_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_A_STAR_HEURISTICS_H
