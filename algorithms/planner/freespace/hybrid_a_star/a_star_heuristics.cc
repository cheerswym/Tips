#include "onboard/planner/freespace/hybrid_a_star/a_star_heuristics.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>

#include "absl/container/flat_hash_set.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"

namespace qcraft {
namespace planner {

struct Node2d {
  Node2d(int x, int y) : grid_x(x), grid_y(y) {
    id = static_cast<unsigned int>(grid_x) +
         (static_cast<unsigned int>(grid_y) << 10);
  }
  int grid_x = 0;
  int grid_y = 0;
  double cost = 0.0;
  unsigned int id = 0;
};

struct Motion {
  int dx;
  int dy;
  double dcost;
};

bool CheckNodeValidity(
    const FreespaceMap &freespace_map,
    const std::vector<std::pair<std::string, Polygon2d>> &stationary_objects,
    double vehicle_radius, int max_x, int max_y, double grid_resolution,
    const Node2d &node) {
  if (node.grid_x < 0 || node.grid_x >= max_x || node.grid_y < 0 ||
      node.grid_y >= max_y) {
    return false;
  }
  const double x =
      freespace_map.region.min_x() + grid_resolution * (0.5 + node.grid_x);
  const double y =
      freespace_map.region.min_y() + grid_resolution * (0.5 + node.grid_y);
  Vec2d pt(x, y);
  for (const auto &named_obj : stationary_objects) {
    if (named_obj.second.DistanceTo(pt) < vehicle_radius) return false;
  }
  for (const auto &boundary : freespace_map.boundaries) {
    if (boundary.segment.DistanceTo(pt) < vehicle_radius) return false;
  }
  return true;
}

bool CheckNodeValidityWithKDTree(const AABox2d &region,
                                 const SegmentMatcherKdtree &segments_kd_tree,
                                 double vehicle_radius, int max_x, int max_y,
                                 double grid_resolution, const Node2d &node) {
  if (node.grid_x < 0 || node.grid_x >= max_x || node.grid_y < 0 ||
      node.grid_y >= max_y) {
    return false;
  }
  const double x = region.min_x() + grid_resolution * (0.5 + node.grid_x);
  const double y = region.min_y() + grid_resolution * (0.5 + node.grid_y);
  constexpr double distance_buffer = 0.5;
  const auto nearby_segments = segments_kd_tree.GetSegmentInRadius(
      x, y, vehicle_radius + distance_buffer);
  for (const auto &seg_ptr : nearby_segments) {
    if (seg_ptr->DistanceTo(Vec2d(x, y)) < vehicle_radius) {
      return false;
    }
  }
  return true;
}

bool AStarHeuristics::GenerateCostMap(
    const std::vector<FreespaceBoundary> &boundaries,
    const std::vector<std::pair<std::string, Polygon2d>> &stationary_objects) {
  if (cost_map_.empty() || cost_map_.front().empty()) return false;
  // Problem set up.
  absl::flat_hash_set<unsigned int> explored_nodes;
  const auto cmp = [](const Node2d &a, const Node2d &b) {
    return a.cost > b.cost;
  };
  std::priority_queue<Node2d, std::vector<Node2d>, decltype(cmp)> open_pq(cmp);
  if (end_grid_x_ < 0 || end_grid_x_ >= cost_map_.size() || end_grid_y_ < 0 ||
      end_grid_y_ >= cost_map_.front().size()) {
    return false;
  }
  Node2d start_node(end_grid_x_, end_grid_y_);
  explored_nodes.emplace(start_node.id);
  open_pq.emplace(std::move(start_node));

  const double sqrt_2 = std::sqrt(2.0);
  const std::vector<Motion> motions = {
      {-1, -1, sqrt_2}, {-1, 0, 1.0},    {-1, 1, sqrt_2}, {0, -1, 1.0},
      {0, 1, 1.0},      {1, -1, sqrt_2}, {1, 0, 1.0},     {1, 1, sqrt_2}};

  // Construct k-D tree of Segment2d.
  std::vector<Segment2d> segments;
  for (const auto &boundary : boundaries) {
    segments.push_back(boundary.segment);
  }
  for (const auto &named_obj : stationary_objects) {
    for (const auto &segment : named_obj.second.line_segments()) {
      segments.push_back(segment);
    }
  }
  SegmentMatcherKdtree segments_kd_tree(std::move(segments));

  // Dijkstra search.
  while (!open_pq.empty()) {
    const auto cur_node = open_pq.top();
    open_pq.pop();
    cost_map_[cur_node.grid_x][cur_node.grid_y] = cur_node.cost;

    for (const auto &motion : motions) {
      const int next_x = cur_node.grid_x + motion.dx;
      const int next_y = cur_node.grid_y + motion.dy;
      Node2d next_node(next_x, next_y);
      if (!CheckNodeValidityWithKDTree(
              region_, segments_kd_tree, vehicle_radius_, cost_map_.size(),
              cost_map_.front().size(), grid_resolution_, next_node)) {
        continue;
      }
      next_node.cost =
          std::min(cost_map_[next_node.grid_x][next_node.grid_y],
                   cur_node.cost + motion.dcost * grid_resolution_);
      if (explored_nodes.find(next_node.id) == explored_nodes.end()) {
        explored_nodes.emplace(next_node.id);
        open_pq.emplace(std::move(next_node));
      }
    }
  }
  return true;
}

double AStarHeuristics::GetHeuristicsCost(double x, double y) const {
  const int row = FloorToInt((x - region_.min_x()) / grid_resolution_);
  const int column = FloorToInt((y - region_.min_y()) / grid_resolution_);
  if (row < 0 || row >= cost_map_.size() || column < 0 ||
      column >= cost_map_.front().size()) {
    return std::numeric_limits<double>::infinity();
  }
  return cost_map_[row][column];
}
}  // namespace planner
}  // namespace qcraft
