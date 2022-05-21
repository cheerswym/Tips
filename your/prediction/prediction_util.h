#ifndef ONBOARD_PREDICTION_PREDICTION_UTIL_H_
#define ONBOARD_PREDICTION_PREDICTION_UTIL_H_

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/traffic_light_manager.h"
#include "onboard/prediction/prediction.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
struct TrajCollisionInfo {
  int collision_index;
  bool can_pass;
};

// Length data for pedestrian, bike, shunfeng, aion, jinlv minibus, polerstar
// bus.
const std::vector<double> kObjectLengthDataPoint = {1.0,   1.57,  2.95,
                                                    4.786, 5.995, 10.48};
const PiecewiseLinearFunction<double, double> kLengthToWheelbasePlf(
    kObjectLengthDataPoint,
    std::vector<double>{0.6, 1.22, 2.1, 2.92, 3.85, 5.89});
const PiecewiseLinearFunction<double, double> kLengthToMaxFrontSteerPlf(
    kObjectLengthDataPoint,
    std::vector<double>{M_PI / 3.0, 0.49, 0.496, 0.546, 0.583, 0.67});
const PiecewiseLinearFunction<double, double> kLengthToCurvaturePlf(
    kObjectLengthDataPoint,
    std::vector<double>{(M_PI / 3.0) / 0.6, 0.49 / 1.22, 0.496 / 2.1,
                        0.546 / 2.92, 0.583 / 3.85, 0.67 / 5.89});
const PiecewiseLinearFunction<double, double> kLengthToMaxLatAccPlf(
    kObjectLengthDataPoint, std::vector<double>{1.0, 2.0, 2.5, 4.0, 3.5, 3.5});

// Align an object's timestamp to current time. It assumes the object moves
// along its current direction with constant acceleration. If the provided
// current time is too different than perception's time, an error message will
// be returned.
absl::Status AlignPerceptionObjectTime(double current_time,
                                       ObjectProto *object);

ObjectProto LerpObjectProto(const ObjectProto &a, const ObjectProto &b,
                            double alpha);

PredictedTrajectoryPointProto LerpPredictedTrajectoryPointProto(
    const PredictedTrajectoryPointProto &a,
    const PredictedTrajectoryPointProto &b, double alpha);

std::vector<ObjectProto> ResampleObjectProtos(
    absl::Span<const ObjectProto> objs, double current_ts, double time_step,
    int max_steps);

// Maybe vulnerable road users
inline bool MaybeVRU(ObjectType type) {
  return (type == OT_PEDESTRIAN || type == OT_MOTORCYCLIST ||
          type == OT_UNKNOWN_MOVABLE || type == OT_UNKNOWN_STATIC ||
          type == OT_CYCLIST);
}

ObjectType GuessType(const ObjectHistory &obj);

// Test if a trajectory proto represents a stationary trajectory.
inline bool IsStationaryTrajectory(const PredictedTrajectoryProto &traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}
// Test if a trajectory represents a stationary trajectory.
inline bool IsStationaryTrajectory(
    const prediction::PredictedTrajectory &traj) {
  return traj.type() == PredictionType::PT_STATIONARY;
}

// Test if a prediction proto represents a stationary trajectory.
inline bool IsStationaryPrediction(const ObjectPredictionProto &pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories(0));
}
// Test if a prediction represents a stationary trajectory.
inline bool IsStationaryPrediction(const prediction::ObjectPrediction &pred) {
  return pred.trajectories().size() == 1 &&
         IsStationaryTrajectory(pred.trajectories()[0]);
}

/*
  Find the distance from query_point to segment (p1, p2). Intentionally not
  using Segment2d here due to its inefficient normalization upon construction.
 */
double PointToSegmentDistance(Vec2d query_point, Vec2d p1, Vec2d p2,
                              Vec2d *closest_point = nullptr);

Box2d BuildObjectBoundingBox(const ObjectProto &object);

std::vector<Vec2d> GetObjectContour(const ObjectProto &object_proto);
Vec2d GetObjectFrontCenter(const ObjectProto &object_proto);
double GetObjectRawHeading(const ObjectProto &object_proto);

// Generate a trajectory point from a stationary perception object.
PredictedTrajectoryPointProto CreateTrajectoryPointFromStationaryObject(
    const ObjectProto &object);

// Remove unlikely prediction results if the probability is less than the
// threshold. `prob_thresh` should in range [0.0, 1.0).
// NOTE: If the threshold is larger than the maximum probability of
//       the predicted trajectories, do nothing.
void RemoveLowProbabilityPrediction(const double prob_thresh,
                                    ObjectPrediction *const object_prediction);

void FillTrajectoryFromRawPoints(const ObjectProto &object_proto,
                                 const std::vector<Vec2d> &raw_points,
                                 PredictionType traj_type,
                                 double prediction_step,
                                 PredictedTrajectory *traj);

/* Function: InstantPredictionForNewObject()
 * -------------------------------------------------------------
 * This function takes in an ObjectProto by reference. Information needed can be
 * extracted from the ObjectProto and the predicted trajectory will be written
 * into the ObjectPredictionProto and returned. When the input proto is missing
 * some information, a Status of UNKNOWN/NOT_FOUND will be returned instead.
 */
absl::StatusOr<ObjectPredictionProto> InstantPredictionForNewObject(
    const ObjectProto &object);

// TODO(lidong): Move vis code to its own file.
/* Function: Visualize()
 * -------------------------------------------------------------
 * This function can be used to draw the result from
 * InstantPredictionForNewObject().
 */
void Visualize(const ObjectPredictionProto &traj, const std::string &canvas);

void GetMultiTimerStats(const ScopedMultiTimer &timer,
                        MultiTimerStatsProto *stats_proto);
void PrintMultiTimerReportStat(const MultiTimerStatsProto &report_proto);
bool IsTrajTwisted(const PredictedTrajectory &traj);

// Find indexes at which an object collide with curb.
// Return {collision_index, can_pass}, The can_pass is true when the trajectory
// can pass the curb.
std::vector<TrajCollisionInfo> FindTrajCurbCollisionIndex(
    absl::Span<const PredictedTrajectoryPoint> traj,
    const SemanticMapManager &semantic_map_manager, const double object_length,
    qcraft::ObjectType object_type);
// Cutoff traj at curb, if cut then return cutoff index.
std::optional<int> CutoffTrajByCurb(
    const SemanticMapManager *semantic_map_manager, const double object_length,
    std::vector<PredictedTrajectoryPoint> *mutable_points);
// Truncate trajectory
inline void TruncateTrajectoryBySize(
    int new_size, std::vector<PredictedTrajectoryPoint> *mutable_points) {
  mutable_points->resize(
      std::min(static_cast<int>(mutable_points->size()), new_size));
}

void CutoffTrajByCurbCollisionIndexes(
    absl::Span<const TrajCollisionInfo> collision_infos, double nominal_brake,
    double kept_horizon, double safe_horizon, PredictedTrajectory *traj);

void CutoffTrajByTrafficLightStatus(
    const TrafficLightManager::TLStateHashMap &tl_state_map,
    const SemanticMapManager &semantic_map_manager, PredictedTrajectory *traj);

bool IsComingToCrossWalk(const SemanticMapManager *semantic_map_manager,
                         const Vec2d &cur_pos, const Vec2d &next_pos);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_UTIL_H_
