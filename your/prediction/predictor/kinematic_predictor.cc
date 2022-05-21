
#include "onboard/prediction/predictor/kinematic_predictor.h"

#include <utility>

#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/kinematic_model.h"
#include "onboard/prediction/util/trajectory_developer.h"
namespace qcraft {
namespace prediction {
namespace {
// Used for cutting off prediction traj by curb
constexpr double kReversedBrakeForTrajCutoffAtCurb = 2.0;  // m/s^2
constexpr double kNormalBrakeForTrajCutoffAtCurb = -2.0;   // m/s^2
}  // namespace
PredictedTrajectory MakeStationaryPrediction(
    const ObjectHistory* obj, double prediction_horizon,
    ObjectPredictionPriority priority) {
  const auto& last_observation = obj->GetHistory()->back().val.object_proto();
  auto traj_pts =
      DevelopStaticTrajectory(ObjectProtoToUniCycleState(last_observation),
                              kPredictionTimeStep, prediction_horizon);
  return PredictedTrajectory(
      /*probability=*/1.0, priority, "Stationary Prediction",
      PredictionType::PT_STATIONARY, /*index=*/0, std::move(traj_pts),
      /*is_reversed=*/false);
}

PredictedTrajectory MakeCTRAPrediction(const ObjectHistory* obj,
                                       double stop_time,
                                       double prediction_horizon,
                                       ObjectPredictionPriority priority,
                                       bool use_acc) {
  const auto& last_observation = obj->GetHistory()->back().val.object_proto();
  auto traj_pts = DevelopCTRATrajectory(
      ObjectProtoToUniCycleState(last_observation), kPredictionTimeStep,
      stop_time, prediction_horizon, use_acc);
  return PredictedTrajectory(/*probability=*/1.0, priority, "CTRA Prediction",
                             PredictionType::PT_CTRA, /*index=*/0,
                             std::move(traj_pts),
                             /*is_reversed=*/false);
}

PredictedTrajectory MakeCYCVPrediction(
    const ObjectHistory* obj, const SemanticMapManager& semantic_map_mgr,
    double prediction_horizon, ObjectPredictionPriority priority) {
  const auto& last_observation = obj->GetHistory()->back().val.object_proto();
  auto traj_pts =
      DevelopCYCVTrajectory(ObjectProtoToUniCycleState(last_observation),
                            kPredictionTimeStep, prediction_horizon,
                            /*is_reversed=*/false);
  PredictedTrajectory predicted_traj = PredictedTrajectory(
      /*probability=*/1.0, priority, "CVCY Prediction", PredictionType::PT_CYCV,
      /*index=*/0, std::move(traj_pts),
      /*is_reversed=*/false);

  // Cut off by curb
  const auto& object_proto = obj->GetHistory()->back().val.object_proto();
  const double object_length = BuildObjectBoundingBox(object_proto).length();
  const auto curb_collision_infos =
      FindTrajCurbCollisionIndex(predicted_traj.points(), semantic_map_mgr,
                                 object_length, object_proto.type());
  CutoffTrajByCurbCollisionIndexes(
      curb_collision_infos, kNormalBrakeForTrajCutoffAtCurb,
      kEmergencyGuardHorizon, kSafeHorizon, &predicted_traj);
  return predicted_traj;
}

PredictedTrajectory MakeReverseCYCVPrediction(
    const ObjectHistory* obj, const SemanticMapManager& semantic_map_mgr,
    double prediction_horizon, ObjectPredictionPriority priority) {
  const auto& last_observation = obj->GetHistory()->back().val.object_proto();
  auto traj_pts =
      DevelopCYCVTrajectory(ObjectProtoToUniCycleState(last_observation),
                            kPredictionTimeStep, prediction_horizon,
                            /*is_reversed=*/true);
  PredictedTrajectory predicted_traj;
  predicted_traj = PredictedTrajectory(
      /*probability=*/1.0, priority, "Reverse CVCY Prediction",
      PredictionType::PT_REVERSE_CYCV, /*index=*/0, std::move(traj_pts),
      /*is_reversed=*/true);

  // Cut off by curb
  const auto& object_proto = obj->GetHistory()->back().val.object_proto();
  const double object_length = BuildObjectBoundingBox(object_proto).length();
  const auto curb_collision_infos =
      FindTrajCurbCollisionIndex(predicted_traj.points(), semantic_map_mgr,
                                 object_length, object_proto.type());
  CutoffTrajByCurbCollisionIndexes(
      curb_collision_infos, kReversedBrakeForTrajCutoffAtCurb,
      kEmergencyGuardHorizon, kSafeHorizon, &predicted_traj);

  return predicted_traj;
}

}  // namespace prediction
}  // namespace qcraft
