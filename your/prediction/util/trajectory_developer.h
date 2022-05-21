
#ifndef ONBOARD_PREDICTION_UTIL_TRAJECTORY_DEVELOPER_H_
#define ONBOARD_PREDICTION_UTIL_TRAJECTORY_DEVELOPER_H_

#include <vector>

#include "absl/strings/string_view.h"
#include "onboard/planner/discretized_path.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/container/prediction_object.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/prediction/util/kinematic_model.h"

namespace qcraft {
namespace prediction {

std::vector<PredictedTrajectoryPoint> DevelopStaticTrajectory(
    const UniCycleState& state, double dt, double horizon);

// Const Turn Rate and Acceleration (CTRA). For now, only for forward maneuver.
// Stop time: the moment which we stop using yaw rate and acceleration value.
std::vector<PredictedTrajectoryPoint> DevelopCTRATrajectory(
    const UniCycleState& state, double dt, double stop_time, double horizon,
    bool use_acc);

// Const Yaw Const Velocity (CYCV)
std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const UniCycleState& state, double dt, double horizon, bool is_reversed);

std::vector<PredictedTrajectoryPoint> DevelopCYCVTrajectory(
    const PredictionObject& obj, double dt, double horizon, bool is_reversed);

absl::StatusOr<std::vector<PredictedTrajectoryPoint>>
DevelopConstVelocityPolePlacementTrajectory(const BicycleModelState& state,
                                            const planner::DrivePassage& dp,
                                            double dt, double horizon,
                                            double obj_len);

std::vector<PredictedTrajectoryPoint> DevelopConstVelocityPurePursuitTrajectory(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    double lane_offset, double preview_t, double min_look_ahead,
    double max_look_ahead, double dt, double horizon, double lf, double lr,
    double max_curvature, bool is_reverse_driving);

planner::DiscretizedPath DevelopPurePursuitPath(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    double lane_offset, double preview_t, double dt, double horizon, double lf,
    double lr, double max_curvature);

planner::DiscretizedPath DevelopPurePursuitPathAtIntersection(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    double lane_offset, double preview_t, double dt, double horizon, double lf,
    double lr, double max_curvature);

planner::SpeedVector DevelopConstVelocitySpeedProfile(double cur_speed,
                                                      double dt,
                                                      double horizon);
std::vector<PredictedTrajectoryPoint>
CombinePathAndSpeedForPredictedTrajectoryPoints(
    const planner::DiscretizedPath& path,
    const planner::SpeedVector& speed_data);

PredictedTrajectory BuildPredictedTrajectory(
    absl::Span<const PredictedTrajectoryPoint> traj_pts,
    absl::string_view annotation, double probability,
    const ObjectPredictionPriority priority, PredictionType pred_type);

BicycleModelState ObjectProtoToBicycleModelState(
    const ObjectProto& object_proto);

double LineFitLateralSpeedByHistory(const ObjectHistorySpan& history,
                                    const planner::DrivePassage& dp);

BicycleModelState UpdateBicycleStatebySteer(const BicycleModelState& state,
                                            double steer_cmd, double length);

double LineFitAccelerationByHistory(const ObjectHistorySpan& history);

planner::DiscretizedPath PredictedTrajectoryPointsToDiscretizedPath(
    absl::Span<const PredictedTrajectoryPoint> points);
}  // namespace prediction
}  // namespace qcraft

#endif
