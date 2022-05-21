#include "onboard/prediction/conflict_resolver/object_st_map.h"

#include <algorithm>
#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/math/vec.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"

namespace qcraft::prediction {
namespace {

// Reuse planner's stboundary related classes.
using StBoundaryRef = planner::StBoundaryRef;  // unique_ptr.
using StBoundary = planner::StBoundary;
using StBoundaryPoints = planner::StBoundaryPoints;

// If the path is behind the half plane by this radius, still consider the half
// plane can be mapped to the path.
constexpr double kRedTLHalfPlaneSearchRadius = 2.0;  // m.

StBoundaryProto::ObjectType ToStBoundaryObjectType(ObjectType type) {
  switch (type) {
    case ObjectType::OT_VEHICLE:
    case ObjectType::OT_UNKNOWN_MOVABLE:
      return StBoundaryProto::VEHICLE;
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
      return StBoundaryProto::CYCLIST;
    case ObjectType::OT_PEDESTRIAN:
      return StBoundaryProto::PEDESTRIAN;
    case ObjectType::OT_UNKNOWN_STATIC:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_CONE:
    case ObjectType::OT_WARNING_TRIANGLE:
      return StBoundaryProto::STATIC;
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
      return StBoundaryProto::IGNORABLE;
  }
}

bool CheckCircleOverlap(
    const Box2d& av_box, const Box2d& object_box,
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    bool check_stationary) {
  const double shrink_ratio =
      check_stationary ? general_config.stationary_object_shrink_ratio()
                       : general_config.dynamic_object_shrink_ratio();
  auto shrinked_object_box = object_box;
  shrinked_object_box.LateralExtendByRatio(shrink_ratio);
  shrinked_object_box.LongitudinalExtendByRatio(shrink_ratio);
  const double distance =
      shrinked_object_box.center().DistanceTo(av_box.center());
  // Av_box circle diameter = box width, object_box is shrinked so radius using
  // half diagonal of the shrinked box.
  return (distance <= 0.5 * (av_box.width() + shrinked_object_box.diagonal()));
}

std::unique_ptr<SegmentMatcherKdtree> BuildKdTree(
    const planner::DiscretizedPath& path) {
  FUNC_QTRACE();
  std::vector<Vec2d> points;
  points.reserve(path.size());
  for (const auto& point : path) {
    points.emplace_back(point.x(), point.y());
  }
  // const auto segs = mapping::Vec2dToSegments(points);
  return std::make_unique<SegmentMatcherKdtree>(std::move(points));
}

std::vector<Box2d> BuildEgoBoxes(const Box2d& av_box, double buffer,
                                 const planner::DiscretizedPath& path) {
  const double half_length = av_box.length() * 0.5 + buffer;
  const double half_width = av_box.width() * 0.5 + buffer;

  // Use path point pos as center of the box.
  std::vector<Box2d> boxes;
  boxes.reserve(path.size());
  for (int i = 0; i < path.size(); ++i) {
    const auto& pt = path[i];
    const Vec2d center(pt.x(), pt.y());
    boxes.push_back(Box2d(half_length, half_width, center, pt.theta()));
  }
  return boxes;
}

absl::Status FindOverLapRange(
    const SegmentMatcherKdtree& path_kd_tree,
    const std::vector<Box2d>& av_boxes_on_path,
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    Vec2d search_point, double radius, const Box2d& object_box,
    double required_lateral_gap, bool check_stationary, int* low_idx,
    int* high_idx) {
  auto indices = path_kd_tree.GetSegmentIndexInRadius(search_point.x(),
                                                      search_point.y(), radius);
  if (indices.empty()) {
    return absl::NotFoundError(
        absl::StrFormat("No point found within %.2f meter radius.", radius));
  }
  std::sort(indices.begin(), indices.end());

  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& raw_av_box = av_boxes_on_path[*it];
    Box2d av_box = raw_av_box;
    av_box.LongitudinalExtend(required_lateral_gap * 2.0);
    av_box.LateralExtend(required_lateral_gap * 2.0);
    if (CheckCircleOverlap(av_box, object_box, general_config,
                           check_stationary)) {
      *low_idx = *it;
      updated = true;
      break;
    }
  }
  if (!updated) {
    return absl::InternalError("search_point has no overlap with path.");
  }

  if (check_stationary) {
    // If checking overlap with stationary object, skip finding the high_idx.
    return absl::OkStatus();
  }

  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& raw_av_box = av_boxes_on_path[*it];
    Box2d av_box = raw_av_box;
    av_box.LongitudinalExtend(required_lateral_gap * 2.0);
    av_box.LateralExtend(required_lateral_gap * 2.0);
    if (CheckCircleOverlap(av_box, object_box, general_config,
                           check_stationary)) {
      *high_idx = *it;
      break;
    }
  }
  return absl::OkStatus();
}

struct StationaryObjectInfo {
  std::string traj_id;
  const PredictedTrajectory* predicted_trajectory = nullptr;
  const ObjectProto* object_proto = nullptr;
  const PredictedTrajectoryPoint* predicted_point = nullptr;
};

absl::StatusOr<StBoundaryPoints> MapStationaryObject(
    const SegmentMatcherKdtree& path_kd_tree,
    const planner::DiscretizedPath& path,
    const std::vector<Box2d>& av_boxes_on_path,
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    const StationaryObjectInfo& object_info, double av_radius,
    double max_plan_time) {
  const auto& object_proto = *object_info.object_proto;
  const auto& predicted_point = *object_info.predicted_point;
  StBoundaryPoints st_boundary_points;
  const Box2d obj_box(object_proto.bounding_box());
  const double search_radius = obj_box.half_width() + av_radius;
  int low_idx = 0;
  int high_idx = path.size() - 1;
  // TODO(changqing): Set required lateral gap for different object type.
  RETURN_IF_ERROR(FindOverLapRange(
      path_kd_tree, av_boxes_on_path, general_config, predicted_point.pos(),
      search_radius, obj_box, /*required_lateral_gap=*/0.0,
      /*check_stationary=*/true, &low_idx, &high_idx));
  const double low_s = path[low_idx].s();
  const double high_s = path.back().s();
  st_boundary_points.lower_points = {{low_s, 0.0}, {low_s, max_plan_time}};
  st_boundary_points.upper_points = {{high_s, 0.0}, {high_s, max_plan_time}};
  st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, max_plan_time}};
  st_boundary_points.overlap_infos = {
      planner::OverlapInfo{.time = 0.0,
                           .obj_idx = 0,
                           .av_start_idx = low_idx,
                           .av_end_idx = high_idx}};
  return st_boundary_points;
}

struct MovingObjectInfo {
  std::string traj_id;
  const ObjectProto* object_proto = nullptr;
  const PredictedTrajectory* predicted_trajectory = nullptr;
};

// From st_graph.cc
double ComputeRelativeSpeed(double pred_point_heading, double pred_point_v,
                            double path_point_heading) {
  const auto path_point_dir = Vec2d::FastUnitFromAngle(path_point_heading);
  const auto pred_point_dir = Vec2d::FastUnitFromAngle(pred_point_heading);
  return pred_point_v * pred_point_dir.dot(path_point_dir);
}

void AddNewBoundaryPoints(const planner::DiscretizedPath& path,
                          const PredictedTrajectoryPoint& predicted_point,
                          int obj_idx, int low_idx, int high_idx,
                          StBoundaryPoints* ptr_boundary_points) {
  auto& points = *ptr_boundary_points;
  const auto& low_pt = path[low_idx];
  const auto& high_pt = path[high_idx];
  const double mid_s = 0.5 * (low_pt.s() + high_pt.s());
  const auto middle_point = path.Evaluate(mid_s);
  const double mid_v = ComputeRelativeSpeed(
      predicted_point.theta(), predicted_point.v(), middle_point.theta());
  points.speed_points.emplace_back(mid_v, predicted_point.t());
  points.lower_points.emplace_back(low_pt.s(), predicted_point.t());
  points.upper_points.emplace_back(high_pt.s(), predicted_point.t());
  points.overlap_infos.emplace_back(
      planner::OverlapInfo{.time = predicted_point.t(),
                           .obj_idx = obj_idx,
                           .av_start_idx = low_idx,
                           .av_end_idx = high_idx});
}

absl::StatusOr<std::vector<StBoundaryPoints>> MapMovingObject(
    const SegmentMatcherKdtree& path_kd_tree,
    const planner::DiscretizedPath& path,
    const std::vector<Box2d>& av_boxes_on_path,
    const ConflictResolutionConfigProto::ConflictResolverConfig& general_config,
    const ConflictResolutionConfigProto::ObjectConflictResolverConfig&
        object_config,
    const MovingObjectInfo& object_info, double av_radius,
    double max_plan_time) {
  // Only map at most one st boundary.
  std::vector<StBoundaryPoints> st_boundary_points;
  const auto& object_proto = *object_info.object_proto;
  const auto& predicted_trajectory = *object_info.predicted_trajectory;
  const auto& predicted_points = predicted_trajectory.points();
  if (predicted_points.size() == 0) {
    return absl::InternalError(
        absl::StrFormat("Predicted points size zero for moving object %s.",
                        object_info.traj_id));
  }
  const Box2d obj_box(object_proto.bounding_box());
  const double search_radius = obj_box.half_width() + av_radius;

  bool prev_state_has_overlap = false;
  // constexpr double kNegativeTimeThrshold = -1e-6;
  constexpr int kEvalStep = 1;
  int low_idx = 0;
  int high_idx = 0;
  for (int i = 0; i < predicted_points.size(); i += kEvalStep) {
    const auto& predicted_point = predicted_points[i];
    const auto overlap_status = FindOverLapRange(
        path_kd_tree, av_boxes_on_path, general_config, predicted_point.pos(),
        search_radius, obj_box, /*required_lateral_gap=*/0.0,
        /*check_stationary=*/false, &low_idx, &high_idx);
    if (!overlap_status.ok()) {
      continue;
    }
    if (!prev_state_has_overlap) {
      st_boundary_points.emplace_back();
      prev_state_has_overlap = true;
    }
    // If prev state has overlap, add new boundary points to the existing.
    AddNewBoundaryPoints(path, predicted_point, i, low_idx, high_idx,
                         &st_boundary_points.back());
  }
  if (st_boundary_points.empty()) {
    return absl::InternalError(
        absl::StrFormat("No overlap when mapping %s.", object_info.traj_id));
  }
  return st_boundary_points;
}

absl::StatusOr<StBoundaryPoints> MapTlRedLane(
    const SegmentMatcherKdtree& path_kd_tree, const SemanticMapManager& smm,
    const planner::DiscretizedPath& path,
    const std::vector<Box2d>& av_boxes_on_path, double av_radius,
    double max_plan_time, const mapping::ElementId& lane_id) {
  // Try mapping a half plane onto the path. Generate a st boundary.
  // Might have unsuccessful mapping due to path length.

  // Get lane point position to create a half plane.
  const auto* lane_info = smm.FindLaneInfoOrNull(lane_id);
  if (lane_info == nullptr) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find Lane %d info. So we can not map this red "
                        "light controlled lane onto st map.",
                        lane_id));
  }
  const auto lane_start = lane_info->LerpPointFromFraction(0.0);
  Vec2d lane_start_tangent;
  if (!lane_info->GetTangent(0.0, &lane_start_tangent)) {
    return absl::InternalError(absl::StrFormat(
        "Cannot find Lane %d start point tangent info.", lane_id));
  }
  // Make sure the path/ego proceeds towards the stopline.
  auto indices = path_kd_tree.GetSegmentIndexInRadiusWithHeading(
      lane_start.x(), lane_start.y(), lane_start_tangent.FastAngle(),
      kRedTLHalfPlaneSearchRadius + av_radius, M_PI_2);
  if (indices.empty()) {
    return absl::InternalError(absl::StrFormat(
        "No close segments around Lane %d red light controlled start point.",
        lane_id));
  }
  std::sort(indices.begin(), indices.end());

  // Create segment2d and check real overlapping range.
  const auto angle = -lane_start_tangent.Perp().Angle();
  const auto unit_vec = Vec2d::FastUnitFromAngle(angle);
  const auto seg =
      Segment2d(lane_start - kRedTLHalfPlaneSearchRadius * unit_vec,
                lane_start + kRedTLHalfPlaneSearchRadius * unit_vec);
  double s_min = path.back().s();
  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& av_box = av_boxes_on_path[*it];
    if (av_box.HasOverlap(seg)) {
      s_min = path[*it].s();
      updated = true;
      break;
    }
  }

  if (!updated) {
    return absl::InternalError(
        absl::StrFormat("Cannot find overlapping s on path with tl red "
                        "controlled point on Lane %d",
                        lane_id));
  }
  s_min = std::max(0.0, s_min);
  const double s_max = path.back().s();
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {planner::StPoint(s_min, 0.0),
                                     planner::StPoint(s_min, max_plan_time)};
  st_boundary_points.upper_points = {planner::StPoint(s_max, 0.0),
                                     planner::StPoint(s_max, max_plan_time)};
  st_boundary_points.speed_points = {planner::VtPoint(0.0, 0.0),
                                     planner::VtPoint(0.0, max_plan_time)};

  return st_boundary_points;
}
}  // namespace

ObjectStMap::ObjectStMap(const ObjectConflictManager& conflict_mgr,
                         const ObjectProto& av_proto,
                         const planner::DiscretizedPath& path,
                         const planner::SpeedVector& ref_speed,
                         const PredictedTrajectory& predicted_trajectory,
                         const ConflictResolverParams* params,
                         ThreadPool* thread_pool) {
  SCOPED_QTRACE("ObjectStMap::ObjectStMap");
  thread_pool_ = thread_pool;
  conflict_mgr_ = &conflict_mgr;
  av_proto_ = &av_proto;
  path_ = &path;
  ref_speed_ = &ref_speed;
  predicted_traj_ = &predicted_trajectory;
  max_length_ = path.length();
  // TODO(changqing): KdTree or BF?
  path_kd_tree_ = BuildKdTree(path);
  const Box2d av_box((*av_proto_).bounding_box());
  general_config_ = &(params->GetGeneralConfig());
  object_config_ = &(params->GetConfigByObjectType(av_proto.type()));
  av_boxes_on_path_ =
      BuildEgoBoxes(av_box, object_config_->ego_box_buffer(), path);
  av_radius_ = 0.5 * av_box.diagonal();
}

absl::Status ObjectStMap::MapStationaryObjects(
    absl::Span<const std::string> traj_ids) {
  SCOPED_QTRACE("ObjectStMap::MapStationaryObjects");
  std::vector<StationaryObjectInfo> infos;
  for (const auto& traj_id : traj_ids) {
    ASSIGN_OR_CONTINUE(const PredictedTrajectory* ptr_traj,
                       conflict_mgr_->GetPredictedTrajectoryByTrajId(traj_id));
    const auto& predicted_trajectory = *ptr_traj;
    if (predicted_trajectory.type() != PT_STATIONARY ||
        predicted_trajectory.points().empty()) {
      continue;
    }
    const auto& predicted_point = predicted_trajectory.points().front();
    ASSIGN_OR_CONTINUE(const ObjectProto* ptr_object_proto,
                       conflict_mgr_->GetObjectProtoByTrajId(traj_id));
    infos.push_back(StationaryObjectInfo({
        .traj_id = traj_id,
        .predicted_trajectory = ptr_traj,
        .object_proto = ptr_object_proto,
        .predicted_point = &predicted_point,
    }));
  }
  std::vector<planner::StBoundaryRef> st_bounds;
  st_bounds.resize(infos.size());
  ParallelFor(0, infos.size(), thread_pool_, [&](int i) {
    const auto& object_info = infos[i];
    const auto st_boundary_points_or = MapStationaryObject(
        *path_kd_tree_, *path_, av_boxes_on_path_, *general_config_,
        object_info, av_radius_, max_plan_time_);
    if (st_boundary_points_or.ok()) {
      st_bounds[i] = planner::StBoundary::CreateInstance(
          *st_boundary_points_or,
          ToStBoundaryObjectType(object_info.object_proto->type()),
          object_info.traj_id, object_info.predicted_trajectory->probability(),
          /*is_stationary=*/true);
    }
  });
  for (auto&& st_bound_ref : st_bounds) {
    if (st_bound_ref == nullptr) continue;
    stationary_objects_.push_back(std::move(st_bound_ref));
  }
  return absl::OkStatus();
}

// TODO(changqing): 5/18 finish mapping and parallelizing. Finish dynamic cost.
// Prepare dev for Thursday.
absl::Status ObjectStMap::MapMovingObjects(
    absl::Span<const std::string> traj_ids) {
  SCOPED_QTRACE("MapMovingObjects");
  std::vector<MovingObjectInfo> infos;
  for (const auto& traj_id : traj_ids) {
    ASSIGN_OR_CONTINUE(const PredictedTrajectory* ptr_traj,
                       conflict_mgr_->GetPredictedTrajectoryByTrajId(traj_id));
    const auto& predicted_trajectory = *ptr_traj;
    if (predicted_trajectory.type() == PT_STATIONARY ||
        predicted_trajectory.points().empty()) {
      continue;
    }
    ASSIGN_OR_CONTINUE(const ObjectProto* ptr_object_proto,
                       conflict_mgr_->GetObjectProtoByTrajId(traj_id));
    infos.push_back(MovingObjectInfo({
        .traj_id = traj_id,
        .object_proto = ptr_object_proto,
        .predicted_trajectory = ptr_traj,
    }));
  }

  std::vector<planner::StBoundaryRef> st_bounds;
  st_bounds.resize(infos.size());
  ParallelFor(0, infos.size(), thread_pool_, [&](int i) {
    const auto& object_info = infos[i];
    const auto st_boundary_points_or = MapMovingObject(
        *path_kd_tree_, *path_, av_boxes_on_path_, *general_config_,
        *object_config_, object_info, av_radius_, max_plan_time_);
    if (st_boundary_points_or.ok()) {
      // Should only have one st boundary.
      st_bounds[i] = planner::StBoundary::CreateInstance(
          st_boundary_points_or->front(),
          ToStBoundaryObjectType(object_info.object_proto->type()),
          object_info.traj_id, object_info.predicted_trajectory->probability(),
          /*is_stationary=*/false);
    }
  });
  for (auto&& st_bound_ref : st_bounds) {
    if (st_bound_ref == nullptr) continue;
    moving_objects_.push_back(std::move(st_bound_ref));
  }
  return absl::OkStatus();
}

void ObjectStMap::MapTLRedLanes(
    const absl::flat_hash_set<mapping::ElementId>& tl_red_lanes,
    absl::Span<const mapping::ElementId> possible_lane_path) {
  for (const auto& lane_id : possible_lane_path) {
    if (!tl_red_lanes.contains(lane_id)) {
      continue;
    }
    const auto st_boundary_points_or = MapTlRedLane(
        *path_kd_tree_, *conflict_mgr_->semantic_map_manager(), *path_,
        av_boxes_on_path_, av_radius_, max_plan_time_, lane_id);
    if (!st_boundary_points_or.ok()) {
      VLOG(3) << absl::StrFormat(
          "Fail to map Lane %d red light controlled point onto path for "
          "object %s traj id %d.",
          lane_id, av_proto_->id(), predicted_traj_->index());
      continue;
    }
    // Add stopline instance onto map.
    stop_lines_.push_back(StBoundary::CreateInstance(
        *st_boundary_points_or, StBoundaryProto::VIRTUAL,
        absl::StrFormat("RED_TL_LANE_%d", lane_id),
        /*probability=*/1.0, /*is_stationary=*/true));
    VLOG(3) << absl::StrFormat(
        "Map Lane %d red light controlled point onto path for "
        "object %s traj id %d.",
        lane_id, av_proto_->id(), predicted_traj_->index());
  }
}

std::vector<double> ObjectStMap::QueryStoplines() const {
  std::vector<double> ret;
  ret.reserve(stop_lines_.size());
  for (const auto& st_bound : stop_lines_) {
    ret.push_back(st_bound->min_s());
  }
  std::sort(ret.begin(), ret.end());
  return ret;
}

std::vector<double> ObjectStMap::QueryStationaryObjects() const {
  std::vector<double> ret;
  ret.reserve(stationary_objects_.size());
  for (const auto& st_bound : stationary_objects_) {
    ret.push_back(st_bound->min_s());
  }
  std::sort(ret.begin(), ret.end());
  return ret;
}

std::vector<const planner::StBoundary*> ObjectStMap::QueryMovingObjects()
    const {
  std::vector<const planner::StBoundary*> ptrs;
  for (const auto& moving_object : moving_objects_) {
    ptrs.push_back(moving_object.get());
  }
  return ptrs;
}

std::string ObjectStMap::DebugString() const {
  std::vector<std::string> infos;
  infos.push_back(
      absl::StrFormat("Object St map for: Object id: %s, Traj index: %d",
                      av_proto_->id(), predicted_traj_->index()));
  infos.push_back(
      absl::StrFormat("Stationary objects mapped: %d, tl stop line mapped: %d",
                      stationary_objects_.size(), stop_lines_.size()));

  for (auto it = stationary_objects_.begin(); it != stationary_objects_.end();
       ++it) {
    infos.push_back((*it)->DebugString());
  }
  for (auto it = stop_lines_.begin(); it != stop_lines_.end(); ++it) {
    infos.push_back((*it)->DebugString());
  }
  return absl::StrJoin(infos, "\n");
}

std::string ObjectStMap::Annotation() const {
  std::vector<std::string> mapped;
  for (auto&& stationary_object : stationary_objects_) {
    mapped.push_back(stationary_object->id());
  }
  for (auto&& tl : stop_lines_) {
    mapped.push_back(tl->id());
  }
  return absl::StrJoin(mapped, ",");
}
}  // namespace qcraft::prediction
