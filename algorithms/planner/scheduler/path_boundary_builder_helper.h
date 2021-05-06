#ifndef ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_
#define ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

class PathBoundary {
 public:
  PathBoundary(std::vector<double> right, std::vector<double> left)
      : right_(std::move(right)), left_(std::move(left)) {}

  const std::vector<double> &right_vec() const { return right_; }

  const std::vector<double> &left_vec() const { return left_; }

  double right(int i) const { return right_[i]; }

  double left(int i) const { return left_[i]; }

  void ExtendLeftTo(double uniform_left) {
    for (auto &left_l : left_) {
      left_l = std::max(left_l, uniform_left);
    }
  }

  void ExtendRightTo(double uniform_right) {
    for (auto &right_l : right_) {
      right_l = std::min(right_l, uniform_right);
    }
  }

  void ShiftLeftBy(double offset) {
    for (auto &left_l : left_) {
      left_l += offset;
    }
  }

  void ShiftRightBy(double offset) {
    for (auto &right_l : right_) {
      right_l += offset;
    }
  }

  void ShiftLeftByIndex(int index, double offset) { left_[index] += offset; }

  void ShiftRightByIndex(int index, double offset) { right_[index] += offset; }

  void OuterClampRightByIndex(int index, double right_l) {
    right_[index] = std::max(right_[index], right_l);
  }

  void OuterClampLeftByIndex(int index, double left_l) {
    left_[index] = std::min(left_[index], left_l);
  }

  void OuterClampBy(const PathBoundary &other) {
    const int n = static_cast<int>(left_.size());
    QCHECK_EQ(n, right_.size());
    QCHECK_EQ(n, other.right_vec().size());
    for (int i = 0; i < n; ++i) {
      left_[i] = std::min(left_[i], other.left(i));
      right_[i] = std::max(right_[i], other.right(i));
    }
  }

  void InnerClampBy(const PathBoundary &other) {
    const int n = static_cast<int>(left_.size());
    QCHECK_EQ(n, right_.size());
    QCHECK_EQ(n, other.size());
    for (int i = 0; i < n; ++i) {
      left_[i] = std::max(left_[i], other.left(i));
      right_[i] = std::min(right_[i], other.right(i));
    }
  }

  int size() const { return left_.size(); }

 private:
  std::vector<double> right_;
  std::vector<double> left_;
};

bool IsTurningLanePath(const PlannerSemanticMapManager &psmm,
                       mapping::ElementId lane_id);

PathBoundary BuildPathBoundaryFromTargetLane(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    bool borrow_lane_boundary);

PathBoundary BuildCurbPathBoundary(const DrivePassage &drive_passage);

PathBoundary BuildSolidPathBoundary(
    const DrivePassage &drive_passage, const FrenetCoordinate &cur_sl,
    const VehicleGeometryParamsProto &vehicle_geom,
    const ApolloTrajectoryPointProto &plan_start_point,
    double target_lane_offset);

PathBoundary BuildPathBoundaryFromAvKinematics(
    const DrivePassage &drive_passage,
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geom,
    const FrenetCoordinate &cur_sl, const FrenetBox &sl_box,
    absl::Span<const double> s_vec, double target_lane_offset,
    double max_lane_change_lat_accel, bool lane_change_pause);

PathBoundary ShrinkPathBoundaryForLaneChangePause(
    const VehicleGeometryParamsProto &vehicle_geom, const FrenetBox &sl_box,
    const LaneChangeStateProto &lc_state, PathBoundary boundary,
    double target_lane_offset);

PathBoundary ShrinkPathBoundaryForObject(
    const DrivePassage &drive_passage,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const ApolloTrajectoryPointProto &plan_start_point,
    absl::Span<const double> s_vec, absl::Span<const double> center_l,
    absl::Span<const Vec2d> center_xy, PathBoundary boundary);

double ComputeTargetLaneOffset(
    const DrivePassage &drive_passage, const FrenetCoordinate &cur_sl,
    const LaneChangeStateProto &lc_state,
    const ApolloTrajectoryPointProto plan_start_point, double half_av_width);

PathSlBoundary BuildPathSlBoundary(const DrivePassage &drive_passage,
                                   std::vector<double> s_vec,
                                   std::vector<double> ref_center_l,
                                   PathBoundary inner_boundary,
                                   PathBoundary outer_boundary);

std::vector<double> ComputeSmoothedReferenceLine(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    const SmoothedReferenceLineResultMap &smooth_result_map);

}  // namespace qcraft::planner

#endif  // ONBOARD_PATH_BOUNDARY_BUILDER_HELPER_H_
