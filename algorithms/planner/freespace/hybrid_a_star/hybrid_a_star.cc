#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"

#include <algorithm>
#include <array>
#include <bitset>
#include <deque>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/time/time.h"
#include "boost/heap/fibonacci_heap.hpp"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/global/trace.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/freespace/hybrid_a_star/a_star_heuristics.h"
#include "onboard/planner/freespace/hybrid_a_star/fast_reeds_shepp.h"
#include "onboard/planner/freespace/hybrid_a_star/reeds_shepp.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

DEFINE_bool(send_sampling_rs_extension_to_canvas, false,
            "Whether to send all sampling path to canvas.");
DEFINE_bool(send_hybrid_a_star_output_path_to_canvas, false,
            "Whether to send hybrid a star path to canvas.");
DEFINE_bool(enable_parking_reverse_plan, true,
            "Whether to exchange start and end of parallel parking or in "
            "narrow scenarios in ha*.");

namespace qcraft {
namespace planner {
namespace {

struct Motion {
  double delta_x;
  double delta_y;
  double delta_theta;
  double steer;
  double s;
  bool forward;
};

struct HybridAStarPath {
  std::vector<std::shared_ptr<Node3d>> nodes;
  double cost = 0.0;
  int ha_star_nodes_num = 0;
};

void SendHybridAStarPathToCanvas(const HybridAStarPath &path,
                                 const std::string name) {
  vis::Canvas &canvas_path = vantage_client_man::GetCanvas(std::move(name));
  for (int i = 0; i + 1 < path.nodes.size(); ++i) {
    const auto color =
        path.nodes[i + 1]->forward() ? vis::Color::kGreen : vis::Color::kRed;
    canvas_path.DrawLine(
        Vec3d(path.nodes[i]->x(), path.nodes[i]->y(), 0.0),
        Vec3d(path.nodes[i + 1]->x(), path.nodes[i + 1]->y(), 0.0), color);
  }
  for (int i = 0; i < path.ha_star_nodes_num; ++i) {
    canvas_path.DrawCircle(Vec3d(path.nodes[i]->x(), path.nodes[i]->y(), 0.0),
                           0.05, vis::Color::kViolet);
  }
  for (int i = path.ha_star_nodes_num; i < path.nodes.size(); ++i) {
    canvas_path.DrawCircle(Vec3d(path.nodes[i]->x(), path.nodes[i]->y(), 0.0),
                           0.05, vis::Color::kOrange);
  }
}

// Returns a pair of (lateral_buffer, longitudinal_buffer).
std::pair<double, double> GetVehicleBufferForBoundary(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const FreespaceBoundary &boundary) {
  constexpr double kDefaultBuffer = -0.5;
  switch (boundary.type) {
    case FreespaceMapProto::CURB:
      if (boundary.near_parking_spot) {
        return std::make_pair(
            hybrid_a_star_params.near_spot_curb_lateral_buffer(),
            hybrid_a_star_params.near_spot_curb_longitudinal_buffer());
      } else {
        return std::make_pair(hybrid_a_star_params.curb_lateral_buffer(),
                              hybrid_a_star_params.curb_longitudinal_buffer());
      }
    case FreespaceMapProto::BARRIER:
      return std::make_pair(hybrid_a_star_params.barrier_lateral_buffer(),
                            hybrid_a_star_params.barrier_longitudinal_buffer());
    case FreespaceMapProto::YELLOW_SOLID_LANE:
      return std::make_pair(
          hybrid_a_star_params.solid_lane_lateral_buffer(),
          hybrid_a_star_params.solid_lane_longitudinal_buffer());
    case FreespaceMapProto::YELLOW_DASHED_LANE:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
    case FreespaceMapProto::WHITE_SOLID_LANE:
      return std::make_pair(
          hybrid_a_star_params.solid_lane_lateral_buffer(),
          hybrid_a_star_params.solid_lane_longitudinal_buffer());
    case FreespaceMapProto::WHITE_DASHED_LANE:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
    case FreespaceMapProto::PARKING_SPOT:
      return std::make_pair(
          hybrid_a_star_params.spot_line_lateral_buffer(),
          hybrid_a_star_params.spot_line_longitudinal_buffer());
    case FreespaceMapProto::PARKING_STOPPER:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
    case FreespaceMapProto::VIRTUAL:
      return std::make_pair(
          hybrid_a_star_params.virtual_boundary_lateral_buffer(),
          hybrid_a_star_params.virtual_boundary_longitudinal_buffer());
    case FreespaceMapProto::OTHER:
      return std::make_pair(kDefaultBuffer, kDefaultBuffer);
  }
  return std::make_pair(kDefaultBuffer, kDefaultBuffer);
}

bool CheckNodeValidity(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const FreespaceMap &freespace_map,
    const std::vector<std::pair<std::string, Polygon2d>> &stationary_objects,
    const Node3d &node) {
  if (node.x() < freespace_map.region.min_x() ||
      node.y() < freespace_map.region.min_y() ||
      node.x() > freespace_map.region.max_x() ||
      node.y() > freespace_map.region.max_y()) {
    return false;
  }
  for (const auto &named_object : stationary_objects) {
    const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
        veh_geo_params, hybrid_a_star_params.object_lateral_buffer(),
        hybrid_a_star_params.object_longitudinal_buffer());
    if (named_object.second.HasOverlap(vehicle_box)) return false;
  }
  for (const auto &boundary : freespace_map.boundaries) {
    const auto buffers =
        GetVehicleBufferForBoundary(hybrid_a_star_params, boundary);
    const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
        veh_geo_params, buffers.first, buffers.second);
    if (vehicle_box.HasOverlap(boundary.segment)) return false;
  }
  for (const auto &boundary : freespace_map.special_boundaries) {
    switch (boundary.type) {
      case SpecialBoundaryType::GEAR_REVERSE_STOPPER: {
        // Only enable when gear reverse.
        if (!node.forward()) {
          const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
              veh_geo_params, /*lateral_buffer=*/0.0,
              /*longitudinal_buffer=*/0.0);
          if (vehicle_box.HasOverlap(boundary.segment)) return false;
        }
      } break;
      case SpecialBoundaryType::SOFT_PARKING_SPOT_LINE:
      case SpecialBoundaryType::CROSSABLE_LANE_LINE:
        break;
    }
  }
  return true;
}

bool CheckNodeValidityWithKDTree(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params, const AABox2d &region,
    const SegmentMatcherKdtree &segments_kd_tree,
    const absl::flat_hash_map<std::string, Polygon2d> &obs_map,
    const absl::flat_hash_map<std::string, FreespaceBoundary> &boundary_map,
    const std::vector<SpecialBoundary> &special_boundaries,
    const Node3d &node) {
  if (node.x() < region.min_x() || node.y() < region.min_y() ||
      node.x() > region.max_x() || node.y() > region.max_y()) {
    return false;
  }
  const Vec2d av_geo_center = GetAvGeometryCenter(
      Vec2d(node.x(), node.y()), Vec2d(node.cos_theta(), node.sin_theta()),
      veh_geo_params);
  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      av_geo_center.x(), av_geo_center.y(), veh_geo_params.length());
  for (const auto &named_segment : nearby_named_segments) {
    const auto iter = obs_map.find(named_segment.second);
    if (iter != obs_map.end()) {
      const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
          veh_geo_params, hybrid_a_star_params.object_lateral_buffer(),
          hybrid_a_star_params.object_longitudinal_buffer());
      if (iter->second.HasOverlap(vehicle_box)) {
        return false;
      }
    } else {
      const auto boundary_iter = boundary_map.find(named_segment.second);
      QCHECK(boundary_iter != boundary_map.end());
      const auto buffers = GetVehicleBufferForBoundary(hybrid_a_star_params,
                                                       boundary_iter->second);
      const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
          veh_geo_params, buffers.first, buffers.second);
      if (vehicle_box.HasOverlap(*named_segment.first)) {
        return false;
      }
    }
  }
  for (const auto &boundary : special_boundaries) {
    switch (boundary.type) {
      case SpecialBoundaryType::GEAR_REVERSE_STOPPER: {
        // Only enable when gear reverse.
        if (!node.forward()) {
          const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
              veh_geo_params, /*lateral_buffer=*/0.0,
              /*longitudinal_buffer=*/0.0);
          if (vehicle_box.HasOverlap(boundary.segment)) return false;
        }
      } break;
      case SpecialBoundaryType::SOFT_PARKING_SPOT_LINE:
      case SpecialBoundaryType::CROSSABLE_LANE_LINE:
        break;
    }
  }
  return true;
}

bool CheckRSExtensionValidityWithKDTree(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params, const AABox2d &region,
    const SegmentMatcherKdtree &segments_kd_tree,
    const absl::flat_hash_map<std::string, Polygon2d> &obs_map,
    const absl::flat_hash_map<std::string, FreespaceBoundary> &boundary_map,
    const std::vector<SpecialBoundary> &special_boundaries,
    const std::vector<std::shared_ptr<Node3d>> &rs_extension) {
  if (rs_extension.empty()) return false;
  for (const auto &node : rs_extension) {
    if (!CheckNodeValidityWithKDTree(hybrid_a_star_params, veh_geo_params,
                                     region, segments_kd_tree, obs_map,
                                     boundary_map, special_boundaries, *node)) {
      return false;
    }
  }
  return true;
}

// Returns a pair of <forward_motions, backward_motions>.
std::pair<std::vector<Motion>, std::vector<Motion>> GenerateMotions(
    const std::vector<double> &steers, double s) {
  std::vector<Motion> forward_motions;
  std::vector<Motion> backward_motions;
  forward_motions.reserve(steers.size() + 1);
  backward_motions.reserve(steers.size() + 1);
  constexpr double kEpsilon = 1e-6;
  for (const double steer : steers) {
    if (std::abs(steer) < kEpsilon) {
      forward_motions.push_back({s, 0.0, 0.0, 0.0, s, true});
      backward_motions.push_back({-s, 0.0, 0.0, 0.0, s, false});
      continue;
    }
    const double delta_theta = steer * s;
    const double sin_theta = std::sin(delta_theta * 0.5);
    const double cos_theta = std::cos(delta_theta * 0.5);
    const double delta_x = 2.0 * sin_theta * cos_theta / steer;
    const double delta_y = 2.0 * sin_theta * sin_theta / steer;
    forward_motions.push_back({delta_x, delta_y, delta_theta, steer, s, true});
    backward_motions.push_back(
        {-delta_x, delta_y, -delta_theta, steer, s, false});
  }
  return std::make_pair(std::move(forward_motions),
                        std::move(backward_motions));
}

std::vector<std::shared_ptr<Node3d>> GenerateNextNodes(
    const std::pair<std::vector<Motion>, const std::vector<Motion>>
        &all_motions,
    const std::vector<double> &all_steers, bool enable_reverse_driving,
    const Node3d &node, double steer_step) {
  constexpr double kEpsilon = 1e-6;

  const auto select_motion = [&all_motions](double motion_steer,
                                            bool forward) -> Motion {
    if (forward) {
      for (const auto &motion : all_motions.first) {
        if (std::abs(motion.steer - motion_steer) < kEpsilon) {
          return motion;
        }
      }
    } else {
      for (const auto &motion : all_motions.second) {
        if (std::abs(motion.steer - motion_steer) < kEpsilon) {
          return motion;
        }
      }
    }
    QLOG(FATAL);  // Unexpected.
    return {0.0, 0.0, 0.0, 0.0, 0.0, true};
  };

  std::vector<Motion> possible_motions;
  for (const double next_steer : all_steers) {
    // If keep direction, the steer change should be no larger than steer_step.
    if (std::abs(node.steer() - next_steer) < steer_step + kEpsilon) {
      possible_motions.push_back(select_motion(next_steer, node.forward()));
    }
    if (!enable_reverse_driving) continue;
    // Don't allow change direction continuously.
    if (!node.prev_node() || node.forward() == node.prev_node()->forward()) {
      // If change direction, the steer change have no limit.
      possible_motions.push_back(select_motion(next_steer, !node.forward()));
    }
  }

  std::vector<std::shared_ptr<Node3d>> res;
  for (int i = 0; i < possible_motions.size(); ++i) {
    const double next_x = node.x() +
                          possible_motions[i].delta_x * node.cos_theta() -
                          possible_motions[i].delta_y * node.sin_theta();
    const double next_y = node.y() +
                          possible_motions[i].delta_x * node.sin_theta() +
                          possible_motions[i].delta_y * node.cos_theta();
    const double next_theta =
        NormalizeAngle(node.theta() + possible_motions[i].delta_theta);
    res.push_back(std::make_shared<Node3d>(next_x, next_y, next_theta));
    res.back()->set_forward(possible_motions[i].forward);
    res.back()->set_steer(possible_motions[i].steer);
  }
  return res;
}

double GetParkingSpotLineCost(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params, const Node3d &node,
    const std::vector<SpecialBoundary> &boundaries) {
  const auto vehicle_box = node.GetVehicleBoundingBoxWithBuffer(
      veh_geo_params, /*lateral_buffer=*/0.0,
      /*longitudinal_buffer=*/0.0);
  double cost = 0.0;
  for (const auto &boundary : boundaries) {
    // Currently only consider spot line
    if (boundary.type == SpecialBoundaryType::SOFT_PARKING_SPOT_LINE &&
        vehicle_box.HasOverlap(boundary.segment)) {
      cost += hybrid_a_star_params.spot_line_weight();
    }
  }
  return cost;
}

double GetTrajectoryCost(const HybridAStarParamsProto &hybrid_a_star_params,
                         const VehicleGeometryParamsProto &veh_geo_params,
                         const std::vector<SpecialBoundary> &boundaries,
                         const Node3d &prev_node, const Node3d &cur_node) {
  constexpr double kEpsilon = 1e-6;
  double res = prev_node.traj_cost();
  if (cur_node.forward()) {
    res += hybrid_a_star_params.search_step() *
           hybrid_a_star_params.forward_gear_weight();
  } else {
    res += hybrid_a_star_params.search_step() *
           hybrid_a_star_params.backward_gear_weight();
  }
  // If prev_node is the start, do not consider gear switch cost.
  if (cur_node.forward() != prev_node.forward() && prev_node.s() > kEpsilon) {
    res += hybrid_a_star_params.gear_switch_weight();
  }
  if (cur_node.forward() == prev_node.forward() &&
      std::abs(cur_node.steer() - prev_node.steer()) > kEpsilon) {
    res += hybrid_a_star_params.steer_change_weight();
  }
  res += std::abs(cur_node.steer()) * hybrid_a_star_params.steer_weight();
  res += GetParkingSpotLineCost(hybrid_a_star_params, veh_geo_params, cur_node,
                                boundaries);
  return res;
}

double GetReedsSheppCost(const HybridAStarParamsProto &hybrid_a_star_params,
                         const FastReedSheppPath &rs_path, bool current_gear) {
  // Currently doesn't distinguish forward and backward weight in heuristics.
  double res =
      rs_path.total_length * hybrid_a_star_params.forward_gear_weight();
  res += static_cast<double>(rs_path.gear_change_num) *
         hybrid_a_star_params.gear_switch_weight();
  if (rs_path.init_gear != current_gear) {
    res += hybrid_a_star_params.gear_switch_weight();
  }
  return res;
}

std::vector<std::shared_ptr<Node3d>> GetReedsSheppExtension(
    const HybridAStarParamsProto &hybrid_a_star_params, double max_steer,
    const Node3d &start, const Node3d &end) {
  std::vector<std::shared_ptr<Node3d>> res;
  const auto rs_path = GetShortestReedsShepp(start, end, max_steer);
  if (!rs_path.ok()) {
    return res;
  }
  // Check if has short path or reverse driving path.
  std::vector<double> segs_lengths;
  for (const double length : rs_path->segs_lengths) {
    // Two segs may have the same direction.
    if (!segs_lengths.empty() && length * segs_lengths.back() > 0.0) {
      segs_lengths.back() += length;
    } else {
      segs_lengths.push_back(length);
    }
    // TODO(Zhuang): Add enable_reverse_driving to RS path generator.
    if (!hybrid_a_star_params.enable_reverse_driving() && length < 0.0) {
      return res;
    }
  }
  constexpr double kMinPathLength = 0.8;
  // The first segment is connected to ha* path end, so if they have the same
  // direction, we don't need to consider the length of this segment.
  const int start_index =
      (start.forward() == (segs_lengths.front() > 0.0) ? 1 : 0);
  for (int i = start_index; i < segs_lengths.size(); ++i) {
    if (std::abs(segs_lengths[i]) < kMinPathLength) {
      return res;
    }
  }
  // If the ha* path end node has changed direction, rs path shouldn't change
  // direction again.
  if (start.prev_node() != nullptr &&
      start.forward() != start.prev_node()->forward() &&
      start.forward() != (rs_path->segs_lengths.front() > 0.0)) {
    return res;
  }

  const auto compute_motion = [&](double length, char type) -> Motion {
    const bool forward = (length > 0.0);
    const double sign = (forward ? 1.0 : -1.0);
    switch (type) {
      case 'L': {
        const double delta_theta = max_steer * length;
        const double sin_theta = fast_math::Sin(delta_theta * 0.5);
        const double cos_theta = fast_math::Cos(delta_theta * 0.5);
        const double delta_x = 2.0 * sin_theta * cos_theta / max_steer;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_steer;
        return {delta_x,   delta_y,          delta_theta,
                max_steer, std::abs(length), forward};
      }
      case 'R': {
        const double delta_theta = -max_steer * length;
        const double sin_theta = fast_math::Sin(delta_theta * 0.5);
        const double cos_theta = fast_math::Cos(delta_theta * 0.5);
        const double delta_x = 2.0 * sin_theta * cos_theta / max_steer;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_steer;
        return {-delta_x,   -delta_y,         delta_theta,
                -max_steer, std::abs(length), forward};
      }
      case 'S':
        return {sign * std::abs(length), 0.0,    0.0, 0.0,
                std::abs(length),        forward};
    }
    QLOG(FATAL);  // Unexpected.
    return {0.0, 0.0, 0.0, 0.0, 0.0, false};
  };

  QCHECK_EQ(rs_path->segs_lengths.size(), rs_path->segs_types.size());
  // Record the last node.
  std::shared_ptr<Node3d> last_node = std::make_shared<Node3d>(start);
  double end_s_of_last_segment = 0.0;
  for (int i = 0; i < rs_path->segs_lengths.size(); ++i) {
    const double length = rs_path->segs_lengths[i];
    const char type = rs_path->segs_types[i];
    Motion motion = compute_motion(
        std::copysign(hybrid_a_star_params.search_step(), length), type);
    double s = 0.0;
    constexpr double kEpsilon = 0.01;
    while (s + kEpsilon < std::abs(length)) {
      double s_increment = hybrid_a_star_params.search_step();
      if (s + s_increment >= std::abs(length)) {
        s_increment = std::abs(length) - s;
        motion = compute_motion(std::copysign(s_increment, length), type);
      }
      s += s_increment;
      const double next_x = last_node->x() +
                            motion.delta_x * last_node->cos_theta() -
                            motion.delta_y * last_node->sin_theta();
      const double next_y = last_node->y() +
                            motion.delta_x * last_node->sin_theta() +
                            motion.delta_y * last_node->cos_theta();
      const double next_theta =
          NormalizeAngle(last_node->theta() + motion.delta_theta);
      res.push_back(std::make_shared<Node3d>(next_x, next_y, next_theta));
      res.back()->set_forward(motion.forward);
      res.back()->set_steer(motion.steer);
      res.back()->set_s(s + end_s_of_last_segment);
      last_node = res.back();
    }
    if (!res.empty()) {
      end_s_of_last_segment = res.back()->s();
    }
  }
  if (res.empty()) return res;
  // Check if the last point is close to end.
  constexpr double kMaxXYError = 0.05;
  constexpr double kMaxThetaError = 0.01;
  const double xy_error =
      Hypot(res.back()->x() - end.x(), res.back()->y() - end.y());
  const double theta_error =
      std::abs(NormalizeAngle(res.back()->theta() - end.theta()));
  if (xy_error > kMaxXYError || theta_error > kMaxThetaError) {
    res.clear();
    VLOG(1) << "RS extension has a big error, xy error = " << xy_error
            << ", theta error = " << theta_error;
  }
  return res;
}

void MaybeUpdatePath(const HybridAStarParamsProto &hybrid_a_star_params,
                     const VehicleGeometryParamsProto &veh_geo_params,
                     const std::vector<SpecialBoundary> &boundaries,
                     FreespaceTaskProto::TaskType task_type,
                     const PathPoint &goal,
                     const std::shared_ptr<Node3d> &final_node,
                     const std::vector<std::shared_ptr<Node3d>> &rs_extension,
                     int sampled_paths_num,
                     std::optional<HybridAStarPath> *current_path) {
  HybridAStarPath path_candidate;
  path_candidate.nodes.reserve(rs_extension.size());
  // Add hybrid a star path.
  auto cur_node = final_node;
  while (cur_node) {
    path_candidate.nodes.push_back(cur_node);
    cur_node = cur_node->prev_node();
  }
  std::reverse(path_candidate.nodes.begin(), path_candidate.nodes.end());
  path_candidate.ha_star_nodes_num = path_candidate.nodes.size();
  const double hybrid_a_star_end_s = path_candidate.nodes.back()->s();
  // Add RS extension.
  auto prev_node = path_candidate.nodes.back();
  for (const auto &node : rs_extension) {
    node->set_s(node->s() + hybrid_a_star_end_s);
    node->set_traj_cost(GetTrajectoryCost(hybrid_a_star_params, veh_geo_params,
                                          boundaries, *prev_node, *node));
    node->set_prev_node(prev_node);
    path_candidate.nodes.push_back(node);
    prev_node = node;
  }
  // Check path validity.
  if (path_candidate.nodes.size() < 2) {
    return;
  }
  // Compute cost.
  path_candidate.cost = path_candidate.nodes.back()->traj_cost();
  // ParallelCost for reverse perpendicular parking. This cost is to make path
  // parallel to parking spot when entering spot.
  constexpr double kEpsilon = 1e-6;
  constexpr double kParallelCostWeight = 10.0;
  constexpr double kParallelCostDist = 4.0;
  if (task_type == FreespaceTaskProto::PERPENDICULAR_PARKING) {
    auto current_node = path_candidate.nodes.back();
    double current_s = current_node->s();
    while (current_node->prev_node()) {
      auto prev_node = current_node->prev_node();
      // Compute cost every search_step because rs node s_step maybe smaller
      // than search_step.
      if (current_s - prev_node->s() >
          hybrid_a_star_params.search_step() - kEpsilon) {
        current_s = prev_node->s();
        // TODO(Zhuang): Maybe weights are different for different theta error.
        path_candidate.cost +=
            std::abs(NormalizeAngle(current_node->theta() - goal.theta())) *
            kParallelCostWeight;
      }
      current_node = prev_node;
      if (path_candidate.nodes.back()->s() - current_node->s() >
          kParallelCostDist) {
        break;
      }
    }
  }
  if (FLAGS_send_sampling_rs_extension_to_canvas) {
    SendHybridAStarPathToCanvas(
        path_candidate,
        absl::StrFormat("freespace/hybrid_a_star_sample/path_%03d",
                        sampled_paths_num));
  }
  // Check if need update.
  if (!current_path->has_value()) {
    *current_path = path_candidate;
    return;
  }
  if (path_candidate.cost < current_path->value().cost) {
    *current_path = path_candidate;
    return;
  }
}

bool CheckNarrowPerpendicularParkingSpot(
    const VehicleGeometryParamsProto &veh_geo_params,
    const SegmentMatcherKdtree &segments_kd_tree,
    const absl::flat_hash_map<std::string, Polygon2d> &obs_map,
    const absl::flat_hash_map<std::string, FreespaceBoundary> &boundary_map,
    const Node3d &node) {
  constexpr double kMinDistThreshold = 0.5;  // meters
  const auto nearby_named_segments = segments_kd_tree.GetNamedSegmentsInRadius(
      node.x(), node.y(), veh_geo_params.length());
  const auto normal_unit = Vec2d(-node.sin_theta(), node.cos_theta());

  const Vec2d av_pose(node.x(), node.y());
  const Vec2d left_shift_pose = av_pose + normal_unit * kMinDistThreshold * 0.5;
  const Vec2d right_shift_pose =
      av_pose - normal_unit * kMinDistThreshold * 0.5;
  const auto left_shift_vehicle_box = GetAvBoxWithBuffer(
      left_shift_pose, node.theta(), veh_geo_params, /*length_buffer=*/0.0,
      /*width_buffer=*/kMinDistThreshold * 0.5);
  const auto right_shift_vehicle_box = GetAvBoxWithBuffer(
      right_shift_pose, node.theta(), veh_geo_params, /*length_buffer=*/0.0,
      /*width_buffer=*/kMinDistThreshold * 0.5);
  bool left_collided = false, right_collided = false;
  for (const auto &named_segment : nearby_named_segments) {
    if (left_collided && right_collided) {
      break;
    }
    const auto iter = obs_map.find(named_segment.second);
    if (iter != obs_map.end()) {
      left_collided |= iter->second.HasOverlap(left_shift_vehicle_box);
      right_collided |= iter->second.HasOverlap(right_shift_vehicle_box);
    } else {
      const auto boundary_iter = boundary_map.find(named_segment.second);
      QCHECK(boundary_iter != boundary_map.end());
      left_collided |= left_shift_vehicle_box.HasOverlap(*named_segment.first);
      right_collided |=
          right_shift_vehicle_box.HasOverlap(*named_segment.first);
    }
  }
  return left_collided && right_collided;
}

struct Comparator {
  bool operator()(const std::shared_ptr<Node3d> &a,
                  const std::shared_ptr<Node3d> &b) const {
    return a->total_cost() > b->total_cost();
  }
};

}  // namespace

absl::StatusOr<std::vector<DirectionalPath>> FindPath(
    const HybridAStarParamsProto &hybrid_a_star_params,
    const VehicleGeometryParamsProto &veh_geo_params,
    const VehicleDriveParamsProto &vehicle_drive_params,
    FreespaceTaskProto::TaskType task_type, const FreespaceMap &freespace_map,
    absl::Span<const SpacetimeObjectTrajectory *const> stalled_object_trajs,
    const PathPoint &start, const PathPoint &end,
    HybridAStartDebugProto *debug_info) {
  SCOPED_QTRACE("FindPath");

  QCHECK_NOTNULL(debug_info);
  // Grid index should be less than 1000 or the hash of grid id may have
  // problem.
  QCHECK_LT(
      freespace_map.region.length() / hybrid_a_star_params.xy_resolution(),
      1000);
  QCHECK_LT(freespace_map.region.width() / hybrid_a_star_params.xy_resolution(),
            1000);
  QCHECK_LT(M_PI * 2.0 / hybrid_a_star_params.theta_resolution(), 1000);

  std::vector<std::pair<std::string, Polygon2d>> stationary_objects;
  for (const auto &traj_ptr : stalled_object_trajs) {
    const std::string id = std::string(traj_ptr->object_id());
    const Polygon2d obj = traj_ptr->contour();
    stationary_objects.push_back(std::make_pair(std::move(id), std::move(obj)));
  }

  const auto start_time = absl::Now();
  // Problem set up.
  // Store each node's handle in open list
  // https://fossies.org/linux/boost/libs/heap/examples/interface.cpp
  boost::heap::fibonacci_heap<std::shared_ptr<Node3d>,
                              boost::heap::compare<Comparator>>
      open_pq;
  absl::flat_hash_map<unsigned int, decltype(open_pq)::handle_type> open_map;
  absl::flat_hash_set<unsigned int> close_set;

  std::shared_ptr<Node3d> final_node = nullptr;
  std::vector<std::shared_ptr<Node3d>> rs_extension;
  std::optional<HybridAStarPath> result_candidate = std::nullopt;

  // Check input.
  auto start_node = std::make_shared<Node3d>(start.x(), start.y(),
                                             NormalizeAngle(start.theta()));
  auto end_node =
      std::make_shared<Node3d>(end.x(), end.y(), NormalizeAngle(end.theta()));

  if (!CheckNodeValidity(hybrid_a_star_params, veh_geo_params, freespace_map,
                         stationary_objects, *start_node)) {
    debug_info->set_status(HybridAStartDebugProto::START_INVALID);
    return absl::UnavailableError("Start pose invalid!");
  }
  if (!CheckNodeValidity(hybrid_a_star_params, veh_geo_params, freespace_map,
                         stationary_objects, *end_node)) {
    debug_info->set_status(HybridAStartDebugProto::GOAL_INVALID);
    return absl::UnavailableError("End pose invalid!");
  }

  // Contruct motions.
  const double max_kappa =
      hybrid_a_star_params.kappa_slack_ratio() *
      GetCenterMaxCurvature(veh_geo_params, vehicle_drive_params);
  // Currently only three steers, the delta_theta produced by these steers
  // should be larger than theta_resolution of hybrid a star, if not, different
  // nodes may occupy the same 3D grid.
  const std::vector<double> possible_steers = {-max_kappa, 0.0, max_kappa};
  const auto possible_motions =
      GenerateMotions(possible_steers, hybrid_a_star_params.search_step());

  // Construct k-D tree.
  const auto kd_tree_start_time = absl::Now();
  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, Polygon2d> obs_map;
  absl::flat_hash_map<std::string, FreespaceBoundary> boundary_map;
  for (const auto &boundary : freespace_map.boundaries) {
    named_segments.push_back(std::make_pair(boundary.id, boundary.segment));
    boundary_map.emplace(boundary.id, boundary);
  }
  for (const auto &named_obj : stationary_objects) {
    named_segments.push_back(std::make_pair(
        named_obj.first,
        Segment2d(Vec2d(named_obj.second.min_x(), named_obj.second.min_y()),
                  Vec2d(named_obj.second.max_x(), named_obj.second.max_y()))));
    obs_map.emplace(named_obj.first, named_obj.second);
  }
  SegmentMatcherKdtree segments_kd_tree(std::move(named_segments));

  // Construct A star cost map.
  AStarHeuristics a_star_heuristics(
      freespace_map.region, hybrid_a_star_params.a_star_resolution(),
      hybrid_a_star_params.a_star_vehicle_radius(), end_node->x(),
      end_node->y());
  if (!a_star_heuristics.GenerateCostMap(freespace_map.boundaries,
                                         stationary_objects)) {
    debug_info->set_status(HybridAStartDebugProto::COST_MAP_FAIL);
    return absl::InternalError("Generate cost map fail!");
  }
  VLOG(2) << "Construct K-D tree and A star cost map time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - kd_tree_start_time);
  const bool exchange_start_end =
      FLAGS_enable_parking_reverse_plan &&
      (task_type == FreespaceTaskProto::PARALLEL_PARKING ||
       (task_type == FreespaceTaskProto::PERPENDICULAR_PARKING &&
        CheckNarrowPerpendicularParkingSpot(veh_geo_params, segments_kd_tree,
                                            obs_map, boundary_map, *end_node)));
  if (exchange_start_end) {
    std::swap(start_node, end_node);
  }
  start_node->SetGridsAndId(freespace_map.region,
                            hybrid_a_star_params.xy_resolution(),
                            hybrid_a_star_params.theta_resolution());
  end_node->SetGridsAndId(freespace_map.region,
                          hybrid_a_star_params.xy_resolution(),
                          hybrid_a_star_params.theta_resolution());
  open_map.emplace(start_node->id(), open_pq.emplace(start_node));
  int iterations = 0;
  int sampled_paths_num = 0;
  int explored_nodes = 0;
  double node_check_time = 0.0;
  double rs_extension_time = 0.0;
  // Hybrid A star search.
  while (!open_pq.empty() && iterations < hybrid_a_star_params.max_iters()) {
    // Get the node with lowest cost.
    const auto current_node = open_pq.top();
    open_pq.pop();
    // Try RS extension.
    const auto rs_extension_start = absl::Now();
    constexpr int kTryEveryNIters = 3;
    // Currently, parking task must use RS extension.
    if (iterations % kTryEveryNIters == 0) {
      const auto rs_extension_nodes = GetReedsSheppExtension(
          hybrid_a_star_params, max_kappa, *current_node, *end_node);
      if (CheckRSExtensionValidityWithKDTree(
              hybrid_a_star_params, veh_geo_params, freespace_map.region,
              segments_kd_tree, obs_map, boundary_map,
              freespace_map.special_boundaries, rs_extension_nodes)) {
        rs_extension = rs_extension_nodes;
        final_node = current_node;
        MaybeUpdatePath(hybrid_a_star_params, veh_geo_params,
                        freespace_map.special_boundaries, task_type, end,
                        final_node, rs_extension, sampled_paths_num,
                        &result_candidate);
        // If we use sampling, ha* should not finish under this condition.
        if (!hybrid_a_star_params.use_sampling_rs() ||
            sampled_paths_num >= hybrid_a_star_params.sampling_rs_max_iters()) {
          break;
        }
        ++sampled_paths_num;
      }
    }
    rs_extension_time +=
        absl::ToDoubleMilliseconds(absl::Now() - rs_extension_start);
    // Check if reach the destination.
    if (Sqr(current_node->x() - end_node->x()) +
                Sqr(current_node->y() - end_node->y()) <
            Sqr(hybrid_a_star_params.goal_xy_tolerance()) &&
        std::abs(NormalizeAngle(current_node->theta() - end_node->theta())) <
            Sqr(hybrid_a_star_params.goal_theta_tolerance())) {
      final_node = current_node;
      MaybeUpdatePath(hybrid_a_star_params, veh_geo_params,
                      freespace_map.special_boundaries, task_type, end,
                      final_node, /*rs_extension=*/{}, sampled_paths_num,
                      &result_candidate);
      // If we use sampling, ha* should not finish under this condition.
      if (!hybrid_a_star_params.use_sampling_rs()) {
        break;
      }
      ++sampled_paths_num;
    }
    // Mark the node as explored.
    close_set.emplace(current_node->id());
    // Explore next nodes.
    const auto next_nodes =
        GenerateNextNodes(possible_motions, possible_steers,
                          hybrid_a_star_params.enable_reverse_driving(),
                          *current_node, max_kappa);
    for (const auto &next_node : next_nodes) {
      // Check if out of boundary or has collision.
      const auto node_check_start = absl::Now();
      if (!CheckNodeValidityWithKDTree(
              hybrid_a_star_params, veh_geo_params, freespace_map.region,
              segments_kd_tree, obs_map, boundary_map,
              freespace_map.special_boundaries, *next_node)) {
        continue;
      }
      node_check_time +=
          absl::ToDoubleMilliseconds(absl::Now() - node_check_start);
      // Set node ID.
      next_node->SetGridsAndId(freespace_map.region,
                               hybrid_a_star_params.xy_resolution(),
                               hybrid_a_star_params.theta_resolution());
      // Check if already in close set.
      if (close_set.find(next_node->id()) != close_set.end()) {
        continue;
      }
      // Current traj cost.
      const double traj_cost = GetTrajectoryCost(
          hybrid_a_star_params, veh_geo_params,
          freespace_map.special_boundaries, *current_node, *next_node);
      // Check if need to push to queue.
      const auto next_node_iter = open_map.find(next_node->id());
      const bool in_open_map = (next_node_iter != open_map.end());
      if (!in_open_map ||
          (in_open_map &&
           traj_cost < (*(next_node_iter->second))->traj_cost())) {
        explored_nodes++;
        // A star cost.
        const double a_star_cost =
            hybrid_a_star_params.forward_gear_weight() *
            a_star_heuristics.GetHeuristicsCost(next_node->x(), next_node->y());
        // RS cost.
        const auto rs_path =
            GetShortestReedsSheppFromTable(*next_node, *end_node, max_kappa);
        double rs_cost = GetReedsSheppCost(hybrid_a_star_params, rs_path,
                                           next_node->forward());
        // Heuristic cost.
        const double heuristic_cost =
            a_star_cost * hybrid_a_star_params.a_star_heuristic_weight() +
            rs_cost * hybrid_a_star_params.rs_heuristic_weight();
        next_node->set_traj_cost(traj_cost);
        next_node->set_heuristic_cost(heuristic_cost);
        next_node->set_total_cost(traj_cost + heuristic_cost);
        next_node->set_s(current_node->s() +
                         hybrid_a_star_params.search_step());
        next_node->set_prev_node(current_node);
        if (in_open_map) {
          // Update the node if it's open.
          open_pq.update(next_node_iter->second, next_node);
        } else {
          // Push to queue and mark it as open.
          open_map.emplace(next_node->id(), open_pq.emplace(next_node));
        }
      }
    }
    iterations++;
  }

  if (!result_candidate.has_value()) {
    QLOG(ERROR) << "Hybrid A star search fail, iterations = " << iterations
                << ", explored_nodes = " << explored_nodes;
    debug_info->set_iters(iterations);
    debug_info->set_status(HybridAStartDebugProto::SEARCH_FAIL);
    return absl::UnavailableError(
        "Hybrid A star search fail after max iterations or explored all "
        "possible nodes.");
  }
  VLOG(2) << "Node validity check time(ms): " << node_check_time;
  VLOG(2) << "RS extension time(ms): " << rs_extension_time;
  VLOG(1) << "Total time(ms): "
          << absl::ToDoubleMilliseconds(absl::Now() - start_time);
  if (FLAGS_send_hybrid_a_star_output_path_to_canvas) {
    SendHybridAStarPathToCanvas(*result_candidate,
                                "freespace/hybrid_a_star_output_path");
  }

  // Extract result.
  auto stitched_path = result_candidate->nodes;
  constexpr int kMaxPathSize = 64;
  std::vector<std::vector<PathPoint>> paths(1);
  std::deque<bool> gears;
  // Start node is always forward, however its real direction is determined by
  // next node.
  stitched_path[0]->set_forward(stitched_path[1]->forward());
  for (int i = 0; i < stitched_path.size(); ++i) {
    PathPoint pt;
    pt.set_x(stitched_path[i]->x());
    pt.set_y(stitched_path[i]->y());
    pt.set_z(0.0);
    pt.set_theta(stitched_path[i]->forward()
                     ? stitched_path[i]->theta()
                     : NormalizeAngle(stitched_path[i]->theta() + M_PI));
    pt.set_kappa(stitched_path[i]->forward() ? stitched_path[i]->steer()
                                             : -stitched_path[i]->steer());
    pt.set_s(stitched_path[i]->s());
    paths.back().push_back(pt);
    // Switch gear.
    if (i > 0 && i + 1 < stitched_path.size() &&
        stitched_path[i]->forward() != stitched_path[i + 1]->forward()) {
      if (paths.size() >= kMaxPathSize) {
        debug_info->set_status(HybridAStartDebugProto::PATH_ABNORMAL);
        return absl::InternalError("Path gear switchs for too many times!");
      }
      gears.push_back(stitched_path[i]->forward());
      paths.push_back({});
    }
  }
  gears.push_back(stitched_path.back()->forward());
  // Insert one more point when gear switchs.
  for (int i = 1; i < paths.size(); ++i) {
    PathPoint pt;
    pt.set_x(paths[i - 1].back().x());
    pt.set_y(paths[i - 1].back().y());
    pt.set_z(0.0);
    pt.set_theta(NormalizeAngle(paths[i - 1].back().theta() + M_PI));
    pt.set_kappa(paths[i].front().kappa());
    pt.set_s(paths[i - 1].back().s());
    paths[i].insert(paths[i].begin(), pt);
  }
  // Reverse path if needed.
  if (exchange_start_end) {
    const double end_s = paths.back().back().s();
    std::reverse(paths.begin(), paths.end());
    for (auto &path : paths) {
      std::reverse(path.begin(), path.end());
      for (auto &pt : path) {
        pt.set_theta(NormalizeAngle(pt.theta() + M_PI));
        pt.set_kappa(-pt.kappa());
        pt.set_s(end_s - pt.s());
      }
    }
    std::reverse(gears.begin(), gears.end());
    for (auto &gear : gears) {
      gear = !gear;
    }
  }
  // Make sure the path s starts from zero.
  for (int i = 0; i < paths.size(); ++i) {
    const double start_s = paths[i][0].s();
    for (int j = 0; j < paths[i].size(); ++j) {
      paths[i][j].set_s(paths[i][j].s() - start_s);
    }
  }

  std::vector<DirectionalPath> res;
  res.reserve(paths.size());
  for (int i = 0; i < paths.size(); ++i) {
    res.push_back({DiscretizedPath(std::move(paths[i])), gears[i]});
  }
  VLOG(2) << "Hybrid A star search success, iterations = " << iterations
          << ", explored_nodes = " << explored_nodes << ", find "
          << sampled_paths_num << " result candidates.";
  debug_info->set_iters(iterations);
  debug_info->set_status(HybridAStartDebugProto::SUCCESS);
  return res;
}
}  // namespace planner
}  // namespace qcraft
