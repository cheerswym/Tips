#include "onboard/planner/scheduler/scheduler_util.h"

#include <algorithm>

#include "onboard/eval/qevent.h"
#include "onboard/global/trace.h"
#include "onboard/planner/planner_defs.h"

DECLARE_int32(planner_lookforward_time_ms);

namespace qcraft::planner {
namespace {

constexpr double kLaneChangeExecutingAggrThreshold = 1.0;
constexpr double kStartPreparingLCDistance = 200.0;  // meters.
constexpr double kForceMergeAggrThreshold = 2.0;

}  // namespace

LaneChangeStateProto MakeNoneLaneChangeState() {
  LaneChangeStateProto proto;
  proto.set_stage(LaneChangeStage::LCS_NONE);
  // The rest fields remain unavaliable.
  return proto;
}

LaneChangeStateProto MakeLaneChangeState(const DrivePassage &drive_passage,
                                         const FrenetBox &ego_frenet_box) {
  const auto ego_frenet_center = ego_frenet_box.center();
  if (std::abs(ego_frenet_center.l) < kMaxLaneKeepLateralOffset) {
    // Close to target lane center, no lane change state.
    return MakeNoneLaneChangeState();
  }

  const auto boundaries =
      drive_passage.QueryEnclosingLaneBoundariesAtS(ego_frenet_center.s);
  // To deal with virtual lanes with no boundaries other than curbs.
  const double lane_boundary_right_offset =
      std::max(boundaries.first->lat_offset, -kMaxHalfLaneWidth);
  const double lane_boundary_left_offset =
      std::min(boundaries.second->lat_offset, kMaxHalfLaneWidth);
  if (lane_boundary_right_offset < ego_frenet_box.l_min &&
      ego_frenet_box.l_max < lane_boundary_left_offset) {
    // Completely within lane path, no lane change state.
    return MakeNoneLaneChangeState();
  }

  if ((lane_boundary_right_offset > ego_frenet_box.l_min &&
       ego_frenet_box.l_max > lane_boundary_right_offset) ||
      (lane_boundary_left_offset > ego_frenet_box.l_min &&
       ego_frenet_box.l_max > lane_boundary_left_offset)) {
    QEVENT_EVERY_N_SECONDS("zixuan", "cross_lane_boundary",
                           /*every_n_seconds=*/10.0, [](QEvent *qevent) {});
  }

  LaneChangeStateProto lc_state;
  lc_state.set_stage(LaneChangeStage::LCS_EXECUTING);
  lc_state.set_lc_left(ego_frenet_box.l_min < lane_boundary_right_offset);
  lc_state.set_entered_target_lane(lc_state.lc_left()
                                       ? ego_frenet_box.l_max > 0.0
                                       : ego_frenet_box.l_min < 0.0);

  return lc_state;
}

void ToSchedulerOutputProto(const SchedulerOutput &output,
                            SchedulerOutputProto *proto) {
  SCOPED_QTRACE("ToSchedulerOutputProto");

  proto->Clear();

  // Start lane id
  proto->set_start_lane_id(output.drive_passage.lane_path().front().lane_id());

  // Path boundary
  const int path_boundary_size = output.sl_boundary.size();
  auto *boundary = proto->mutable_path_boundary();
  boundary->mutable_reference_center()->Reserve(path_boundary_size);
  boundary->mutable_left_boundary()->Reserve(path_boundary_size);
  boundary->mutable_right_boundary()->Reserve(path_boundary_size);
  boundary->mutable_target_left_boundary()->Reserve(path_boundary_size);
  boundary->mutable_target_right_boundary()->Reserve(path_boundary_size);

  for (const auto &pt : output.sl_boundary.reference_center_xy_vector()) {
    Vec2dToProto(pt, boundary->add_reference_center());
  }
  for (const auto &pt : output.sl_boundary.left_xy_vector()) {
    Vec2dToProto(pt, boundary->add_left_boundary());
  }
  for (const auto &pt : output.sl_boundary.right_xy_vector()) {
    Vec2dToProto(pt, boundary->add_right_boundary());
  }
  for (const auto &pt : output.sl_boundary.target_left_xy_vector()) {
    Vec2dToProto(pt, boundary->add_target_left_boundary());
  }
  for (const auto &pt : output.sl_boundary.target_right_xy_vector()) {
    Vec2dToProto(pt, boundary->add_target_right_boundary());
  }

  // Reasons:
  proto->mutable_reasons()->Reserve(output.reasons.size());
  for (const auto &str : output.reasons) {
    *proto->add_reasons() = str;
  }
}

double ComputeLCExecutingAggressiveness(double rest_distance,
                                        const FrenetBox &ego_frenet_box,
                                        bool lc_left) {
  if (rest_distance < kMinLcLaneLength) return kForceMergeAggrThreshold + 0.01;

  // In range [1.0, 2.0].
  const double nearest_l =
      std::abs(lc_left ? ego_frenet_box.l_max : ego_frenet_box.l_min);

  return std::max(0.0, (1.0 - nearest_l / kDefaultLaneWidth)) +
         kLaneChangeExecutingAggrThreshold;
}

double ComputeLCPreparingAggressiveness(double rest_distance) {
  if (rest_distance > kStartPreparingLCDistance) return 0.0;
  if (rest_distance < kMinLcLaneLength) return kForceMergeAggrThreshold + 0.01;

  // In range [0.0, 1.0].
  return kLaneChangeExecutingAggrThreshold *
         (kStartPreparingLCDistance - rest_distance) /
         (kStartPreparingLCDistance - kMinLcLaneLength);
}

}  // namespace qcraft::planner
