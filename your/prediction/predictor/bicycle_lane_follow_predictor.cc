
#include "onboard/prediction/predictor/bicycle_lane_follow_predictor.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "onboard/global/trace.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/predictor/kinematic_predictor.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kMaxHeadingDiff = M_PI / 3.0;         // rad,
constexpr double kSafeGuardHeadingDiff = M_PI / 12.0;  // rad,
constexpr double kMaxDistanceLimit = 1.5;              // m.
constexpr double kLookAheadTime = 15.0;                // s.
constexpr double kMinPathLength = 20.0;                // m.
constexpr double kBackwardLength = 20.0;               // m.
constexpr double kPathStepS = 2.0;                     // m.
constexpr double kPurePursuitPreviewTime = 1.5;        // s.
constexpr double kMinLookAhead = 1.0;                  // s.
constexpr double kMaxLookAhead = 5.0;                  // s.
constexpr double kBikeNominalBrake = -2.5;             // m/s^2.
constexpr double kMaxCurvature = 0.2;

std::vector<PredictedTrajectory> DevelopBicycleLaneFollowPrediction(
    const ObjectHistory* obj, const PredictionContext& context,
    ObjectPredictionPriority priority) {
  const auto hist_or = obj->GetHistory();
  QCHECK_OK(hist_or.status());
  const auto& hist = hist_or.value();
  const auto& last_obj = hist.back().val;
  const auto& pos = last_obj.pos();
  const auto& semantic_map_mgr = *context.semantic_map_manager();

  const auto lane_id_or = FindNearestLaneIdWithBoundaryDistanceLimit(
      semantic_map_mgr, pos, kMaxDistanceLimit);

  auto ctra_traj_pts = DevelopCTRATrajectory(
      ObjectProtoToUniCycleState(last_obj.object_proto()), kPredictionTimeStep,
      kEmergencyGuardHorizon, kSafeHorizon, /*use_acc=*/false);

  // No lane association, use a simple CTRA prediction.
  if (lane_id_or == std::nullopt) {
    return {PredictedTrajectory(
        /*probability=*/1.0, priority,
        "Bike lane follow: CTRA (no nearest lane)",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }

  ASSIGN_OR_DIE(
      const auto closest_lane_pt,
      FindClosestLanePointToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          semantic_map_mgr.GetLevel(), semantic_map_mgr, pos,
          std::vector<mapping::ElementId>({*lane_id_or}), /*angle=*/0.0,
          /*heading_penality_weight=*/0.0));
  double lane_heading = closest_lane_pt.ComputeLerpTheta(semantic_map_mgr);
  const double angle_diff =
      std::fabs(NormalizeAngle(last_obj.heading() - lane_heading));
  // Almost perpendicular to lane, do not associate them, Use a CTRA prediction
  if (angle_diff > kMaxHeadingDiff && angle_diff < M_PI - kMaxHeadingDiff) {
    return {PredictedTrajectory(
        /*probability=*/1.0, priority,
        "Bike lane follow: CTRA (large heading error)",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }

  // Check if obj is reversed driving or need a safe guard CTRA prediction
  bool is_reversed_driving = false;
  bool need_ctra = false;
  if (angle_diff > M_PI - kMaxHeadingDiff) {
    is_reversed_driving = true;
  }
  if (angle_diff > kSafeGuardHeadingDiff &&
      angle_diff < M_PI - kSafeGuardHeadingDiff) {
    need_ctra = true;
  }
  const auto lane_id = lane_id_or.value();
  const double forward_len =
      std::max(kLookAheadTime * last_obj.v(), kMinPathLength);
  // only consider most straight path.
  auto lane_path =
      SearchMostStraightLanePath(pos, *context.semantic_map_manager(), lane_id,
                                 forward_len, is_reversed_driving);
  const auto obj_box = BuildObjectBoundingBox(last_obj.object_proto());
  const double half_len = obj_box.half_length();

  mapping::LanePath extended_lp;
  if (is_reversed_driving) {
    extended_lp = planner::BackwardExtendLanePath(
        *context.semantic_map_manager(), lane_path, kBackwardLength);
  } else {
    extended_lp = planner::ForwardExtendLanePath(
        *context.semantic_map_manager(), lane_path, kBackwardLength);
  }
  const auto dp = planner::BuildDrivePassageFromLanePath(
      *context.semantic_map_manager(), extended_lp, kPathStepS,
      /*avoid_loop=*/true);
  const auto sl_or = dp.QueryFrenetCoordinateAt(pos);
  double lateral_offset = 0.0;
  if (sl_or.ok()) {
    lateral_offset = sl_or->l;
  }
  auto generated_points = DevelopConstVelocityPurePursuitTrajectory(
      ObjectProtoToBicycleModelState(last_obj.object_proto()), dp,
      lateral_offset, kPurePursuitPreviewTime, kMinLookAhead, kMaxLookAhead,
      kPredictionTimeStep, kComfortableHorizon, half_len, half_len,
      kMaxCurvature, is_reversed_driving);
  if (generated_points.size() <
      static_cast<int>(kEmergencyGuardHorizon / kPredictionTimeStep)) {
    return {PredictedTrajectory(
        /*probability=*/1.0, priority,
        "Bike lane follow: CTRA (not enough pure pursuit points)",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false)};
  }

  std::vector<PredictedTrajectory> trajs;
  trajs.reserve(2);
  double prob = 1.0;
  if (need_ctra) {
    prob = 0.5;
    trajs.push_back(PredictedTrajectory(
        prob, priority, "Bike lane follow: CTRA for safety",
        PredictionType::PT_BIKE_LANE_FOLLOW, 0, std::move(ctra_traj_pts),
        /*is_reversed=*/false));
  }
  trajs.push_back(PredictedTrajectory(
      prob, priority, "Bike lane follow: normal",
      PredictionType::PT_BIKE_LANE_FOLLOW, 1, std::move(generated_points),
      /*is_reversed=*/false));
  return trajs;
}

}  // namespace
std::vector<PredictedTrajectory> MakeBicycleLaneFollowPrediction(
    const ObjectHistory* obj, const PredictionContext& context,
    ObjectPredictionPriority priority) {
  SCOPED_QTRACE("MakeBicycleLaneFollowPrediction");
  auto trajs = DevelopBicycleLaneFollowPrediction(obj, context, priority);
  const auto hist_or = obj->GetHistory();
  QCHECK_OK(hist_or.status());
  const auto& hist = hist_or.value();
  const double object_length = hist.bounding_box().length();
  for (auto& traj : trajs) {
    const auto curb_collision_infos = FindTrajCurbCollisionIndex(
        traj.points(), *context.semantic_map_manager(), object_length,
        qcraft::ObjectType::OT_CYCLIST);
    CutoffTrajByCurbCollisionIndexes(curb_collision_infos, kBikeNominalBrake,
                                     kEmergencyGuardHorizon, kSafeHorizon,
                                     &traj);
  }
  return trajs;
}
}  // namespace prediction
}  // namespace qcraft
