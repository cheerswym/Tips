#include "onboard/planner/scheduler/path_boundary_builder.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/scheduler/path_boundary_builder_helper.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

constexpr double kMaxInnerBoundLaneChangeLatAccel = 0.09;  // m/s^2.
constexpr double kMaxOuterBoundLaneChangeLatAccel = 0.08;  // m/s^2.
constexpr double kExtendOuterBoundWidth = kDefaultHalfLaneWidth;

PathBoundary ExtendBoundaryBy(const DrivePassage &drive_passage,
                              const LaneChangeStateProto &lc_state,
                              PathBoundary boundary, double extend_width) {
  const double left_extend_path_boundary_width =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING && lc_state.lc_left()
          ? 0.0
          : extend_width;
  const double right_extend_path_boundary_width =
      lc_state.stage() == LaneChangeStage::LCS_EXECUTING && !lc_state.lc_left()
          ? 0.0
          : extend_width;

  for (int i = 0; i < boundary.size(); ++i) {
    const auto &station = drive_passage.station(StationIndex(i));
    boundary.ShiftLeftByIndex(i, left_extend_path_boundary_width);
    if (station.direction() == mapping::LaneProto::UTURN &&
        FLAGS_planner_enable_runtime_uturn_task) {
      continue;
    }
    boundary.ShiftRightByIndex(i, -right_extend_path_boundary_width);
  }

  return boundary;
}

std::vector<double> PostprocessOuterBoundary(absl::Span<const double> s_vec,
                                             absl::Span<const double> inner_vec,
                                             std::vector<double> outer_vec) {
  constexpr double kEpsilon = 0.1;
  constexpr double kMinContinuousExtendedBoundLength = 3.0;  // m.
  int first_extended_idx = -1;
  for (int i = 1; i < inner_vec.size(); ++i) {
    bool curr_is_extended = std::fabs(inner_vec[i] - outer_vec[i]) >
                            (kExtendOuterBoundWidth - kEpsilon);
    bool prev_is_extended = std::fabs(inner_vec[i - 1] - outer_vec[i - 1]) >
                            (kExtendOuterBoundWidth - kEpsilon);
    if (first_extended_idx == -1) {
      if (curr_is_extended && !prev_is_extended) {
        first_extended_idx = i;
      }
    } else if (!curr_is_extended) {
      if ((s_vec[i - 1] - s_vec[first_extended_idx]) <=
          kMinContinuousExtendedBoundLength) {
        for (int j = first_extended_idx; j < i; ++j) {
          outer_vec[j] = inner_vec[first_extended_idx - 1];
        }
      }
      first_extended_idx = -1;
    }
  }

  return outer_vec;
}

}  // namespace

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromDrivePassage(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage) {
  const int n = drive_passage.size();
  std::vector<double> s_vec, center_l(n, 0.0);
  s_vec.reserve(n);
  for (const auto &station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());
  }
  auto inner_boundary = BuildPathBoundaryFromTargetLane(
      psmm, drive_passage, /*borrow_lane_boundary=*/false);
  PathBoundary outer_boundary = inner_boundary;
  outer_boundary.ShiftLeftBy(kExtendOuterBoundWidth);
  outer_boundary.ShiftRightBy(-kExtendOuterBoundWidth);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);
  inner_boundary.OuterClampBy(curb_boundary);
  outer_boundary.OuterClampBy(curb_boundary);

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(inner_boundary),
                             std::move(outer_boundary));
}

absl::StatusOr<PathSlBoundary> BuildPathBoundaryFromPose(
    const PlannerSemanticMapManager &psmm, const DrivePassage &drive_passage,
    const ApolloTrajectoryPointProto &plan_start_point,
    const VehicleGeometryParamsProto &vehicle_geom,
    const SpacetimeTrajectoryManager &st_traj_mgr,
    const LaneChangeStateProto &lc_state,
    const SmoothedReferenceLineResultMap &smooth_result_map,
    bool borrow_lane_boundary, bool should_smooth) {
  SCOPED_QTRACE("BuildPathBoundaryFromPose");
  const Box2d ego_box =
      GetAvBox(Vec2dFromApolloTrajectoryPointProto(plan_start_point),
               plan_start_point.path_point().theta(), vehicle_geom);

  ASSIGN_OR_RETURN(const auto sl_box, drive_passage.QueryFrenetBoxAt(ego_box),
                   _ << "Fail to project ego box on drive passage.");

  ASSIGN_OR_RETURN(const auto cur_sl,
                   drive_passage.QueryFrenetCoordinateAt(
                       Vec2dFromApolloTrajectoryPointProto(plan_start_point)),
                   _ << "Fail to project ego pos on drive passage.");

  const int n = drive_passage.stations().size();
  std::vector<double> s_vec;
  s_vec.reserve(n);
  for (const auto &station : drive_passage.stations()) {
    s_vec.push_back(station.accumulated_s());
  }

  std::vector<double> center_l(n, 0.0);
  if (should_smooth && FLAGS_planner_save_smooth_result_in_planner_state) {
    center_l =
        ComputeSmoothedReferenceLine(psmm, drive_passage, smooth_result_map);
  }

  const auto target_lane_offset =
      ComputeTargetLaneOffset(drive_passage, cur_sl, lc_state, plan_start_point,
                              vehicle_geom.width() * 0.5);
  bool lane_change_pause = lc_state.stage() == LaneChangeStage::LCS_PAUSE;
  if (lane_change_pause) {
    const auto cur_station_index =
        drive_passage
            .FindNearestStationIndex(
                Vec2dFromApolloTrajectoryPointProto(plan_start_point))
            .value();
    const auto smoothed_center_offset =
        target_lane_offset - center_l[cur_station_index];
    for (int i = 0; i < n; ++i) {
      center_l[i] += smoothed_center_offset;
    }
  }

  auto boundary = BuildPathBoundaryFromTargetLane(psmm, drive_passage,
                                                  borrow_lane_boundary);

  const auto solid_boundary =
      BuildSolidPathBoundary(drive_passage, cur_sl, vehicle_geom,
                             plan_start_point, target_lane_offset);

  const auto curb_boundary = BuildCurbPathBoundary(drive_passage);

  // Kinematic boundaries assuming constant lateral acceleration.
  const auto inner_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      target_lane_offset, kMaxInnerBoundLaneChangeLatAccel, lane_change_pause);
  const auto outer_kinematic_boundary = BuildPathBoundaryFromAvKinematics(
      drive_passage, plan_start_point, vehicle_geom, cur_sl, sl_box, s_vec,
      target_lane_offset, kMaxOuterBoundLaneChangeLatAccel, lane_change_pause);
  PathBoundary outer_boundary = boundary;
  boundary.InnerClampBy(inner_kinematic_boundary);
  outer_boundary.InnerClampBy(outer_kinematic_boundary);

  // Extend path boundary by half lane width.
  if (!(borrow_lane_boundary || lane_change_pause)) {
    outer_boundary =
        ExtendBoundaryBy(drive_passage, lc_state, std::move(outer_boundary),
                         kExtendOuterBoundWidth);
  }

  // Shrink when lane change pause.
  if (lane_change_pause) {
    outer_boundary = ShrinkPathBoundaryForLaneChangePause(
        vehicle_geom, sl_box, lc_state, std::move(outer_boundary),
        target_lane_offset);
    outer_boundary.InnerClampBy(inner_kinematic_boundary);
    boundary.OuterClampBy(outer_boundary);
  }

  // Clamp boundary by curb and solid line.
  if (!borrow_lane_boundary) {
    boundary.OuterClampBy(solid_boundary);
    outer_boundary.OuterClampBy(solid_boundary);
    boundary.InnerClampBy(inner_kinematic_boundary);
    outer_boundary.InnerClampBy(outer_kinematic_boundary);
  }
  boundary.OuterClampBy(curb_boundary);
  outer_boundary.OuterClampBy(curb_boundary);

  // Post-process center_l to make sure it is within path boundary.
  for (int i = 0; i < n; ++i) {
    if (boundary.left(i) - boundary.right(i) <= vehicle_geom.width()) {
      center_l[i] = (boundary.left(i) + boundary.right(i)) * 0.5;
    } else {
      center_l[i] = std::clamp(center_l[i],
                               boundary.right(i) + vehicle_geom.width() * 0.5,
                               boundary.left(i) - vehicle_geom.width() * 0.5);
    }
  }

  // Shrink to fit objects.
  std::vector<Vec2d> center_xy;
  for (int i = 0; i < n; ++i) {
    const auto &station = drive_passage.station(StationIndex(i));
    center_xy.push_back(station.lat_point(center_l[i]));
  }
  if (lc_state.stage() != LaneChangeStage::LCS_EXECUTING ||
      lc_state.entered_target_lane()) {
    outer_boundary = ShrinkPathBoundaryForObject(
        drive_passage, st_traj_mgr, plan_start_point, s_vec, center_l,
        center_xy, std::move(outer_boundary));
    outer_boundary.InnerClampBy(boundary);
  }

  // Post-process outer boundary to avoid zigzags.
  outer_boundary =
      PathBoundary(PostprocessOuterBoundary(s_vec, boundary.right_vec(),
                                            outer_boundary.right_vec()),
                   PostprocessOuterBoundary(s_vec, boundary.left_vec(),
                                            outer_boundary.left_vec()));
  outer_boundary.InnerClampBy(boundary);

  return BuildPathSlBoundary(drive_passage, std::move(s_vec),
                             std::move(center_l), std::move(boundary),
                             std::move(outer_boundary));
}

}  // namespace qcraft::planner
