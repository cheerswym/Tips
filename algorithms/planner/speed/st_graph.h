#ifndef ONBOARD_PLANNER_SPEED_ST_GRAPH_H_
#define ONBOARD_PLANNER_SPEED_ST_GRAPH_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "onboard/math/geometry/common_shapes.h"
#include "onboard/math/geometry/kdtree.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_params.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct StDistancePoint {
  double t = 0.0;
  double path_s = 0.0;
  double distance = 0.0;
  double relative_v = 0.0;
};

struct CloseSpaceTimeObject {
  std::vector<StDistancePoint> st_distance_points;
  bool is_stationary = false;
  bool is_away_from_traj = false;
  StBoundaryProto::ObjectType object_type;
  std::string id;
  Box2d box;
};

class StGraph {
 public:
  explicit StGraph(const DiscretizedPath& path_points, bool forward,
                   int traj_steps,
                   const VehicleGeometryParamsProto* vehicle_geo_params,
                   const SpeedFinderParamsProto* speed_finder_params);

  std::vector<StBoundaryRef> GetStBoundaries(
      absl::Span<const SpacetimeObjectTrajectory* const>
          moving_spacetime_objects,
      absl::Span<const SpacetimeObjectTrajectory* const>
          stationary_spacetime_objects,
      const ConstraintManager& constraint_mgr,
      const qcraft::mapping::SemanticMapManager& sman_mgr,
      ThreadPool* thread_pool) const;

  // For moving object, this function will only check and return the first
  // point(which is the object current state) if it is within the
  // slow_down_radius.
  std::vector<CloseSpaceTimeObject> GetCloseSpaceTimeObjects(
      absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
      absl::Span<const SpacetimeObjectTrajectory* const> spacetime_object_trajs,
      double slow_down_radius) const;

  std::vector<StBoundaryRef> MapMovingSpacetimeObject(
      const SpacetimeObjectTrajectory& spacetime_object) const;

 private:
  std::vector<StBoundaryRef> MapStationarySpacetimeObjects(
      absl::Span<const SpacetimeObjectTrajectory* const>
          stationary_spacetime_objs) const;

  StBoundaryRef MapStopLine(
      const ConstraintProto::StopLineProto& stop_line) const;

  StBoundaryRef MapNearestImpassableBoundary(
      const qcraft::mapping::SemanticMapManager& sman_mgr) const;

  void BuildKdTree(const DiscretizedPath& path_points);

  // Generate possibly multiple st-boundaries for a moving object.
  absl::StatusOr<std::vector<StBoundaryPoints>> GetMovingObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object) const;

  // Generate single st-boundary for a stationary object.
  absl::StatusOr<StBoundaryPoints> GetStationaryObjStBoundaryPoints(
      const SpacetimeObjectTrajectory& spacetime_object) const;

  absl::Status FindOverlapRange(Vec2d search_point, double radius,
                                const Polygon2d& obj_shape,
                                double required_lateral_gap, int* low_idx,
                                int* high_idx) const;

  absl::Status FindStopLineOverlapRange(
      const ConstraintProto::StopLineProto& stop_line, int* low_idx,
      int* high_idx) const;

  bool GetStDistancePointInfo(const SpacetimeObjectState& state,
                              double slow_down_radius,
                              StDistancePoint* st_distance_point) const;

 private:
  double total_plan_time_ = 0.0;
  DiscretizedPath path_points_;
  bool forward_ = true;
  // The SDC shapes on path points.
  std::vector<Box2d> av_box_on_path_ponits_;
  std::unique_ptr<SegmentMatcherKdtree> path_kd_tree_;
  double ego_radius_ = 0.0;

  const VehicleGeometryParamsProto* vehicle_geo_params_;
  const SpeedFinderParamsProto* speed_finder_params_;
  const SpeedFinderParamsProto::StGraphParamsProto* st_graph_param_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ST_GRAPH_H_
