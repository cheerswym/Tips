#include "onboard/planner/speed/st_graph.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>

#include "absl/status/status.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"

DEFINE_bool(planner_enable_multiple_st_boundaries, true,
            "Whether enable multiple st_boundaries for single st_object.");

namespace qcraft::planner {

namespace {

using ::qcraft::prediction::PredictedTrajectory;

// Build AV polygons for each path point.
std::vector<Box2d> BuildAvBoxes(const VehicleGeometryParamsProto& vehicle_geom,
                                double buffer,
                                const DiscretizedPath& path_points,
                                bool forward) {
  const double half_length = vehicle_geom.length() * 0.5 + buffer;
  const double half_width = vehicle_geom.width() * 0.5 + buffer;
  const double rac_to_center =
      half_length - (buffer + vehicle_geom.back_edge_to_center());
  std::vector<Box2d> av_boxes;
  const int num_points = path_points.size();
  av_boxes.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const auto& pt = path_points[i];
    const double theta =
        forward ? pt.theta() : NormalizeAngle(pt.theta() + M_PI);
    const Vec2d rac(pt.x(), pt.y());
    const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
    const Vec2d center = rac + tangent * rac_to_center;
    av_boxes.push_back(
        Box2d(half_length, half_width, center, pt.theta(), tangent));
  }
  return av_boxes;
}

inline bool CheckOverlap(const Box2d& av_box, const Polygon2d& obj_shape) {
  return obj_shape.HasOverlap(av_box);
}

absl::Status CheckStBoundary(const StBoundary& st_boundary) {
  const auto& id = st_boundary.id();
  absl::Status status = absl::OkStatus();
  if (st_boundary.lower_points().size() < 2) {
    status = absl::InternalError(absl::StrFormat(
        "st_boundary: %s lower points size is less than 2.", id));
    QEVENT("ping", "st_graph_invalid_st_boundary", [&status](QEvent* qevent) {
      qevent->AddField("error_reason", status.message());
    });
    return status;
  }

  if (st_boundary.max_s() < 0.0) {
    status = absl::InternalError(
        absl::StrFormat("st_boundary: %s max_s() < 0.0.", id));
  }

  if (st_boundary.max_t() < 0.0) {
    status = absl::InternalError(
        absl::StrFormat("st_boundary: %s max_t() < 0.0.", id));
  }

  if (!status.ok()) {
    VLOG(2) << status.ToString();
    QEVENT("ping", "st_graph_invalid_st_boundary", [&status](QEvent* qevent) {
      qevent->AddField("error_reason", status.message());
    });
  }

  return absl::OkStatus();
}

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

double ComputeRelativeSpeed(double pred_point_heading, double pred_point_v,
                            double path_point_heading) {
  const auto path_point_dir = Vec2d::FastUnitFromAngle(path_point_heading);
  const auto pred_point_dir = Vec2d::FastUnitFromAngle(pred_point_heading);
  return pred_point_v * pred_point_dir.dot(path_point_dir);
}

// Check if we want to use perception object's bounding box to check for
// overlap.
bool DecideIfUsePerceptionBox(const SpacetimeObjectTrajectory& traj,
                              const PathPoint& av_point) {
  if (traj.planner_object()->type() != ObjectType::OT_VEHICLE) {
    return false;
  }

  QCHECK_GT(traj.states().size(), 0);
  const auto& first_state = traj.states()[0];
  const Vec2d av_tan = Vec2d::FastUnitFromAngle(av_point.theta());
  const Vec2d obj_center = first_state.contour.CircleCenter();
  const bool is_completely_behind = av_tan.Dot(obj_center - ToVec2d(av_point)) <
                                    -first_state.contour.CircleRadius();
  return is_completely_behind;
}

void CheckAndRemoveInvalidStBoundaryPoints(
    std::vector<StBoundaryPoints>* boundaries_points) {
  QCHECK_NOTNULL(boundaries_points);
  QCHECK(!boundaries_points->empty());
  const auto& points = boundaries_points->back();
  QCHECK(!points.lower_points.empty());
  QCHECK_EQ(points.lower_points.size(), points.upper_points.size());
  if (points.lower_points.size() < 2) boundaries_points->pop_back();
}

bool IsMappableSpacetimeObject(
    const SpacetimeObjectTrajectory& spacetime_object) {
  if (spacetime_object.states().empty()) {
    VLOG(2) << "Skip mapping spacetime object " << spacetime_object.traj_id()
            << " because its states are empty.";
    return false;
  }
  if (ToStBoundaryObjectType(spacetime_object.planner_object()->type()) ==
      StBoundaryProto::IGNORABLE) {
    VLOG(2) << "Skip mapping spacetime object " << spacetime_object.traj_id()
            << " because its object type "
            << ObjectType_Name(spacetime_object.planner_object()->type())
            << " is ignorable.";
    return false;
  }
  return true;
}

std::string MakeStBoundaryId(absl::string_view traj_id, int idx) {
  return absl::StrFormat("%s|%d", traj_id, idx);
}

}  // namespace

// StGraph.
StGraph::StGraph(const DiscretizedPath& path_points, bool forward,
                 int traj_steps,
                 const VehicleGeometryParamsProto* vehicle_geo_params,
                 const SpeedFinderParamsProto* speed_finder_params)
    : total_plan_time_(kTrajectoryTimeStep * traj_steps),
      path_points_(path_points),
      forward_(forward),
      vehicle_geo_params_(QCHECK_NOTNULL(vehicle_geo_params)),
      speed_finder_params_(QCHECK_NOTNULL(speed_finder_params)),
      st_graph_param_(&speed_finder_params_->st_graph_params()) {
  SCOPED_QTRACE("StGraph::StGraph");

  BuildKdTree(path_points_);
  av_box_on_path_ponits_ =
      BuildAvBoxes(*vehicle_geo_params, st_graph_param_->boundary_buffer(),
                   path_points_, forward_);
  ego_radius_ = Hypot(std::max(vehicle_geo_params_->front_edge_to_center(),
                               vehicle_geo_params_->back_edge_to_center()),
                      vehicle_geo_params_->right_edge_to_center());
}

std::vector<StBoundaryRef> StGraph::GetStBoundaries(
    absl::Span<const SpacetimeObjectTrajectory* const> moving_spacetime_objects,
    absl::Span<const SpacetimeObjectTrajectory* const>
        stationary_spacetime_objects,
    const ConstraintManager& constraint_mgr, const SemanticMapManager& sman_mgr,
    ThreadPool* thread_pool) const {
  SCOPED_QTRACE("EstPlanner/GetStBoundaries");

  std::vector<StBoundaryRef> st_boundaries;

  if (path_points_.size() < 2) {
    QLOG(WARNING) << "Fail to get st_boundary because of too few path points.";
    return st_boundaries;
  }

  // Map spacetime objects on st_graph.
  const int num_workers =
      thread_pool == nullptr ? 0 : thread_pool->NumWorkers();
  std::vector<std::vector<StBoundaryRef>> st_boundaries_for_workers(
      num_workers + 1);

  ParallelFor(0, moving_spacetime_objects.size(), thread_pool,
              [&](parallel_for::WorkerIndex worker_index, int i) {
                if (IsMappableSpacetimeObject(*moving_spacetime_objects[i])) {
                  auto moving_st_boundaries =
                      MapMovingSpacetimeObject(*moving_spacetime_objects[i]);
                  for (int j = 0; j < moving_st_boundaries.size(); ++j) {
                    if (CheckStBoundary(*moving_st_boundaries[j]).ok()) {
                      st_boundaries_for_workers[worker_index].push_back(
                          std::move(moving_st_boundaries[j]));
                    }
                  }
                }
              });

  // Collect all st_boundaries from each workers.
  for (auto& single_worker_st_boundaries : st_boundaries_for_workers) {
    st_boundaries.insert(
        st_boundaries.end(),
        std::make_move_iterator(single_worker_st_boundaries.begin()),
        std::make_move_iterator(single_worker_st_boundaries.end()));
  }

  // Map nearest stationary object on st_graph.
  auto stationary_st_boundaries =
      MapStationarySpacetimeObjects(stationary_spacetime_objects);
  for (int i = 0; i < stationary_st_boundaries.size(); ++i) {
    if (CheckStBoundary(*stationary_st_boundaries[i]).ok()) {
      VLOG(3) << "Map stationary objects on st_graph, id: "
              << stationary_st_boundaries[i]->id();
      st_boundaries.push_back(std::move(stationary_st_boundaries[i]));
    }
  }

  // Map stop_line on st_graph.
  const int num_stop_lines = constraint_mgr.StopLine().size();
  std::vector<StBoundaryRef> st_boundaries_from_stop_lines(num_stop_lines);
  ParallelFor(0, num_stop_lines, thread_pool, [&](int i) {
    const auto& stop_line = constraint_mgr.StopLine()[i];
    if (StBoundaryRef st_boundary = MapStopLine(stop_line)) {
      if (CheckStBoundary(*st_boundary).ok()) {
        st_boundaries_from_stop_lines[i] = std::move(st_boundary);
        VLOG(3) << "Map stop line on st_graph, id: " << stop_line.id();
      }
    }
  });
  for (auto& st_boundary : st_boundaries_from_stop_lines) {
    if (st_boundary != nullptr) {
      st_boundaries.push_back(std::move(st_boundary));
    }
  }

  // Map nearest collision impassable boundary on st_graph.
  if (auto nearest_impassable_st_boundary =
          MapNearestImpassableBoundary(sman_mgr)) {
    if (CheckStBoundary(*nearest_impassable_st_boundary).ok()) {
      VLOG(3) << "Map impassable boundary on st_graph, id: "
              << nearest_impassable_st_boundary->id();
      st_boundaries.push_back(std::move(nearest_impassable_st_boundary));
    }
  }

  return st_boundaries;
}

StBoundaryRef StGraph::MapNearestImpassableBoundary(
    const SemanticMapManager& sman_mgr) const {
  bool is_collision = false;
  constexpr double kCollisionBuffer = 0.05;
  constexpr double kSearchRadiusBuffer = 0.2;

  double low_s = 0.0;
  std::string id;
  for (const auto& path_point : path_points_) {
    const Box2d ego_box = GetAvBoxWithBuffer(
        ToVec2d(path_point),
        forward_ ? path_point.theta()
                 : NormalizeAngle(path_point.theta() + M_PI),
        *vehicle_geo_params_, kCollisionBuffer, kCollisionBuffer);
    const double search_radius = ego_box.diagonal() * 0.5 + kSearchRadiusBuffer;
    const auto named_segs = sman_mgr.GetNamedImpassableBoundariesAtLevel(
        sman_mgr.GetLevel(), ego_box.center(), search_radius);
    for (const auto& named_seg : named_segs) {
      if (ego_box.HasOverlap(named_seg.first)) {
        is_collision = true;
        low_s = path_point.s();
        id = named_seg.second;
        break;
      }
    }
    if (is_collision) break;
  }
  if (!is_collision) return nullptr;

  const double s_min = std::max(0.0, low_s);
  const double s_max = path_points_.back().s();
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};

  return QCHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::IMPASSABLE_BOUNDARY, id,
      /*probability=*/1.0,
      /*is_stationary=*/true));
}

absl::Status StGraph::FindStopLineOverlapRange(
    const ConstraintProto::StopLineProto& stop_line, int* low_idx,
    int* high_idx) const {
  QCHECK_NOTNULL(low_idx);
  QCHECK_NOTNULL(high_idx);

  const HalfPlane half_plane(stop_line.half_plane());
  const auto half_plane_seg = Segment2d(half_plane.start(), half_plane.end());
  const auto center = half_plane.center();
  const double radius = half_plane.length() * 0.5 + ego_radius_;

  const double driving_direction = (-half_plane.tangent().Perp()).FastAngle();
  auto indices = path_kd_tree_->GetSegmentIndexInRadiusWithHeading(
      center.x(), center.y(), driving_direction, radius, M_PI_2);
  if (indices.empty()) {
    return absl::InternalError(
        absl::StrCat("No point found for stopline id: ", stop_line.id()));
  }
  std::sort(indices.begin(), indices.end());

  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& av_box = av_box_on_path_ponits_[*it];
    if (av_box.HasOverlap(half_plane_seg)) {
      *low_idx = *it;
      updated = true;
      break;
    }
  }
  if (!updated) {
    return absl::InternalError(absl::StrCat("Stop line ", stop_line.id(),
                                            " has no overlap with path."));
  }
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& av_box = av_box_on_path_ponits_[*it];
    if (av_box.HasOverlap(half_plane_seg)) {
      *high_idx = *it;
      break;
    }
  }
  return absl::OkStatus();
}

StBoundaryRef StGraph::MapStopLine(
    const ConstraintProto::StopLineProto& stop_line) const {
  double s_min = 0.0;
  double s_max = 0.0;
  if (stop_line.id() == "end_of_local_path") {
    QCHECK_GE(stop_line.s(), 0.0);
    QCHECK_LE(stop_line.s(), path_points_.back().s());
    s_min = stop_line.s();
    s_max = path_points_.back().s();
  } else {
    int low_idx = 0;
    int high_idx = 0;
    const auto status =
        FindStopLineOverlapRange(stop_line, &low_idx, &high_idx);
    if (!status.ok()) {
      VLOG(3) << status.ToString() << " stopline id: " << stop_line.id();
      return nullptr;
    }

    s_min = std::max(0.0, path_points_[low_idx].s());
    s_max = std::max(s_min, path_points_.back().s());
  }
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(s_min, 0.0),
                                     StPoint(s_min, total_plan_time_)};
  st_boundary_points.upper_points = {StPoint(s_max, 0.0),
                                     StPoint(s_max, total_plan_time_)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0),
                                     VtPoint(0.0, total_plan_time_)};

  return QCHECK_NOTNULL(StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, stop_line.id(),
      /*probability=*/1.0,
      /*is_stationary=*/true));
}

std::vector<StBoundaryRef> StGraph::MapMovingSpacetimeObject(
    const SpacetimeObjectTrajectory& spacetime_object) const {
  QCHECK(!spacetime_object.is_stationary());

  std::vector<StBoundaryRef> st_boundaries;
  std::vector<StBoundaryPoints> st_boundaries_points;

  auto st_boundaries_points_or = GetMovingObjStBoundaryPoints(spacetime_object);
  if (st_boundaries_points_or.ok()) {
    st_boundaries_points = std::move(*st_boundaries_points_or);
  } else {
    return st_boundaries;
  }

  const int size = st_boundaries_points.size();
  if (!FLAGS_planner_enable_multiple_st_boundaries) QCHECK_EQ(size, 1);

  st_boundaries.reserve(size);
  for (int i = 0; i < size; ++i) {
    const std::string st_boundary_id =
        size == 1 ? std::string(spacetime_object.traj_id())
                  : MakeStBoundaryId(spacetime_object.traj_id(), i);
    QCHECK_EQ(st_boundaries_points[i].lower_points.size(),
              st_boundaries_points[i].upper_points.size());
    if (st_boundaries_points[i].lower_points.size() < 2) continue;
    auto st_boundary = StBoundary::CreateInstance(
        st_boundaries_points[i],
        ToStBoundaryObjectType(spacetime_object.planner_object()->type()),
        st_boundary_id, spacetime_object.trajectory()->probability(),
        /*is_stationary=*/false);
    st_boundaries.push_back(std::move(st_boundary));
  }
  return st_boundaries;
}

std::vector<StBoundaryRef> StGraph::MapStationarySpacetimeObjects(
    absl::Span<const SpacetimeObjectTrajectory* const>
        stationary_spacetime_objs) const {
  std::vector<StBoundaryRef> st_boundaries;
  std::vector<StBoundaryPoints> st_boundaries_points;

  for (int i = 0; i < stationary_spacetime_objs.size(); ++i) {
    const SpacetimeObjectTrajectory* obj = stationary_spacetime_objs[i];
    QCHECK(obj->is_stationary());
    if (!IsMappableSpacetimeObject(*obj)) continue;
    const std::string traj_id(obj->traj_id());
    auto st_boundary_points_or = GetStationaryObjStBoundaryPoints(*obj);
    if (!st_boundary_points_or.ok()) {
      VLOG(3) << st_boundary_points_or.status().ToString()
              << " id: " << traj_id;
      continue;
    }
    st_boundaries.push_back(
        StBoundary::CreateInstance(std::move(*st_boundary_points_or),
                                   ToStBoundaryObjectType(obj->object_type()),
                                   traj_id, obj->trajectory()->probability(),
                                   /*is_stationary=*/true));
  }

  return st_boundaries;
}

absl::StatusOr<StBoundaryPoints> StGraph::GetStationaryObjStBoundaryPoints(
    const SpacetimeObjectTrajectory& spacetime_object) const {
  QCHECK(spacetime_object.is_stationary());
  StBoundaryPoints st_boundary_points;

  const Box2d& obj_box = spacetime_object.bounding_box();
  const double search_radius = obj_box.diagonal() * 0.5 + ego_radius_;

  int low_idx = 0;
  int high_idx = 0;
  RETURN_IF_ERROR(FindOverlapRange(
      obj_box.center(), search_radius, spacetime_object.contour(),
      spacetime_object.required_lateral_gap(), &low_idx, &high_idx));

  const double low_s = path_points_[low_idx].s();
  // Set high_s to be path length.
  const double high_s = path_points_.back().s();
  st_boundary_points.lower_points = {{low_s, 0.0}, {low_s, total_plan_time_}};
  st_boundary_points.upper_points = {{high_s, 0.0}, {high_s, total_plan_time_}};
  st_boundary_points.speed_points = {{0.0, 0.0}, {0.0, total_plan_time_}};
  st_boundary_points.overlap_infos = {OverlapInfo{.time = 0.0,
                                                  .obj_idx = 0,
                                                  .av_start_idx = low_idx,
                                                  .av_end_idx = high_idx}};
  return st_boundary_points;
}

absl::StatusOr<std::vector<StBoundaryPoints>>
StGraph::GetMovingObjStBoundaryPoints(
    const SpacetimeObjectTrajectory& spacetime_object) const {
  QCHECK(!spacetime_object.is_stationary());
  std::vector<StBoundaryPoints> st_boundaries_points;

  const Box2d& obj_box = spacetime_object.bounding_box();
  const double search_radius = obj_box.diagonal() * 0.5 + ego_radius_;

  const auto& object_states = spacetime_object.states();
  QCHECK_GT(object_states.size(), 0);
  const bool use_perception_box =
      DecideIfUsePerceptionBox(spacetime_object, path_points_[0]);

  bool prev_state_has_overlap = false;
  constexpr double kNegtiveTimeThreshold = -1e-6;
  int low_idx = 0;
  int high_idx = 0;
  for (int i = 0; i < object_states.size(); ++i) {
    const auto& cur_state = object_states[i];
    const auto* traj_point = cur_state.traj_point;
    QCHECK_GT(traj_point->t(), kNegtiveTimeThreshold);
    const auto obj_contour = use_perception_box
                                 ? Polygon2d(cur_state.perception_box)
                                 : cur_state.contour;
    if (!FindOverlapRange(cur_state.box.center(), search_radius, obj_contour,
                          spacetime_object.required_lateral_gap(), &low_idx,
                          &high_idx)
             .ok()) {
      if (FLAGS_planner_enable_multiple_st_boundaries) {
        if (prev_state_has_overlap) {
          CheckAndRemoveInvalidStBoundaryPoints(&st_boundaries_points);
        }
        prev_state_has_overlap = false;
      }
      continue;
    }
    if (!prev_state_has_overlap) st_boundaries_points.emplace_back();
    prev_state_has_overlap = true;

    auto& boundary_points = st_boundaries_points.back();
    const double middle_s =
        (path_points_[high_idx].s() + path_points_[low_idx].s()) * 0.5;
    const auto middle_point = path_points_.Evaluate(middle_s);
    const double middle_speed = ComputeRelativeSpeed(
        traj_point->theta(), traj_point->v(), middle_point.theta());
    const double low_s = path_points_[low_idx].s();
    const double high_s = path_points_[high_idx].s();
    boundary_points.speed_points.emplace_back(middle_speed, traj_point->t());
    boundary_points.lower_points.emplace_back(low_s, traj_point->t());
    boundary_points.upper_points.emplace_back(high_s, traj_point->t());
    boundary_points.overlap_infos.emplace_back(
        OverlapInfo{.time = traj_point->t(),
                    .obj_idx = i,
                    .av_start_idx = low_idx,
                    .av_end_idx = high_idx});
  }
  if (st_boundaries_points.empty()) {
    return absl::InternalError("No overlap.");
  }
  return st_boundaries_points;
}

absl::Status StGraph::FindOverlapRange(Vec2d search_point, double radius,
                                       const Polygon2d& obj_shape,
                                       double required_lateral_gap,
                                       int* low_idx, int* high_idx) const {
  QCHECK_NOTNULL(low_idx);
  QCHECK_NOTNULL(high_idx);

  auto indices = path_kd_tree_->GetSegmentIndexInRadius(
      search_point.x(), search_point.y(), radius);
  if (indices.empty()) {
    return absl::NotFoundError(
        absl::StrFormat("No point found within %.2f meter radius.", radius));
  }
  std::sort(indices.begin(), indices.end());

  bool updated = false;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    const auto& raw_av_box = av_box_on_path_ponits_[*it];
    Box2d av_box = raw_av_box;
    av_box.LongitudinalExtend(required_lateral_gap * 2.0);
    av_box.LateralExtend(required_lateral_gap * 2.0);
    if (CheckOverlap(av_box, obj_shape)) {
      *low_idx = *it;
      updated = true;
      break;
    }
  }
  if (!updated) {
    return absl::InternalError("search_point has no overlap with path.");
  }
  for (auto it = indices.rbegin(); it != indices.rend(); ++it) {
    const auto& raw_av_box = av_box_on_path_ponits_[*it];
    Box2d av_box = raw_av_box;
    av_box.LongitudinalExtend(required_lateral_gap * 2.0);
    av_box.LateralExtend(required_lateral_gap * 2.0);
    if (CheckOverlap(av_box, obj_shape)) {
      *high_idx = *it;
      break;
    }
  }
  return absl::OkStatus();
}

void StGraph::BuildKdTree(const DiscretizedPath& path_points) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& point : path_points) {
    points.emplace_back(point.x(), point.y());
  }
  const auto segs = mapping::Vec2dToSegments(points);
  path_kd_tree_ = std::make_unique<SegmentMatcherKdtree>(std::move(segs));
}

bool StGraph::GetStDistancePointInfo(const SpacetimeObjectState& state,
                                     double slow_down_radius,
                                     StDistancePoint* st_distance_point) const {
  QCHECK_NOTNULL(st_distance_point);
  const Box2d& obj_box = state.box;
  const double search_radius =
      obj_box.diagonal() * 0.5 + ego_radius_ + slow_down_radius;
  const auto indexes = path_kd_tree_->GetSegmentIndexInRadius(
      obj_box.center().x(), obj_box.center().y(), search_radius);
  if (indexes.empty()) {
    return false;
  }

  double min_dist = std::numeric_limits<double>::max();
  PathPoint nearest_pt;
  for (const auto index : indexes) {
    const auto& pt = path_points_[index];
    const double dist = obj_box.DistanceTo(ToVec2d(pt));
    if (dist < min_dist) {
      min_dist = dist;
      nearest_pt = pt;
    }
  }

  // The distance to the object is approximate.
  min_dist -= vehicle_geo_params_->width() * 0.5;
  if (min_dist > slow_down_radius) {
    return false;
  }

  st_distance_point->path_s = nearest_pt.s();
  st_distance_point->distance = min_dist;
  st_distance_point->relative_v = ComputeRelativeSpeed(
      (*state.traj_point).theta(), (*state.traj_point).v(), nearest_pt.theta());
  return true;
}

std::vector<CloseSpaceTimeObject> StGraph::GetCloseSpaceTimeObjects(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
    double slow_down_radius) const {
  absl::flat_hash_set<std::string> traj_id_set;
  traj_id_set.reserve(st_boundaries_with_decision.size());
  for (const auto& st_boundary_with_decision : st_boundaries_with_decision) {
    if (const auto& traj_id = st_boundary_with_decision.traj_id();
        traj_id.has_value()) {
      traj_id_set.insert(*traj_id);
    }
  }
  std::vector<CloseSpaceTimeObject> close_space_time_objects;
  for (const SpacetimeObjectTrajectory* obj_traj : spacetime_object_trajs) {
    if (traj_id_set.contains(obj_traj->traj_id()) ||
        obj_traj->states().empty()) {
      continue;
    }
    if (ToStBoundaryObjectType(obj_traj->planner_object()->type()) ==
        StBoundaryProto::IGNORABLE) {
      continue;
    }

    StDistancePoint st_dis_point;
    if (!GetStDistancePointInfo(obj_traj->states()[0], slow_down_radius,
                                &st_dis_point)) {
      continue;
    }

    auto& object = close_space_time_objects.emplace_back();
    object.st_distance_points.push_back(std::move(st_dis_point));
    object.is_stationary = obj_traj->is_stationary();
    // To check if the moving object is moving away from our traj.
    if (!object.is_stationary) {
      QCHECK_GE(obj_traj->states().size(), 2);
      StDistancePoint second_st_dis_point;
      if (!GetStDistancePointInfo(obj_traj->states()[1], slow_down_radius,
                                  &second_st_dis_point)) {
        object.is_away_from_traj = true;
      } else {
        if (second_st_dis_point.distance - st_dis_point.distance >
            qcraft::prediction::kPredictionTimeStep *
                std::fabs(st_dis_point.relative_v)) {
          object.is_away_from_traj = true;
        } else {
          object.is_away_from_traj = false;
        }
      }
    }
    object.box = obj_traj->states()[0].box;
    object.object_type =
        ToStBoundaryObjectType(obj_traj->planner_object()->type());
    object.id = obj_traj->traj_id();
  }
  return close_space_time_objects;
}

}  // namespace qcraft::planner
