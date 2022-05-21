#ifndef ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_RESULT_H_
#define ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_RESULT_H_
#include <string>
#include <vector>

#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {
struct ObjectPredictionResult {
  ObjectIDType id;                    // object id.
  ObjectPredictionPriority priority;  // Priority of object.
  ObjectPredictionScenario scenario;  // Scenario of object.
  ObjectProto perception_object;      // Perception object.
  StopTimeInfo stop_time_info;        // How long did this object stop.
  ObjectLongTermBehaviorProto
      long_term_behavior;  // Long term behavior of objects.
  std::vector<PredictedTrajectory> trajectories;  // predicted trajectories.

  void ToProto(ObjectPredictionProto* obj_pred) const {
    obj_pred->set_id(id);
    for (const auto& traj : trajectories) {
      traj.ToProto(obj_pred->add_trajectories());
    }
    auto& stop_time = *obj_pred->mutable_stop_time();
    stop_time.set_time_duration_since_stop(
        stop_time_info.time_duration_since_stop());
    stop_time.set_previous_stop_time_duration(
        stop_time_info.previous_stop_time_duration());
    stop_time.set_last_move_time_duration(
        stop_time_info.last_move_time_duration());
    *obj_pred->mutable_perception_object() = perception_object;
    *obj_pred->mutable_long_term_behavior() = long_term_behavior;
  }
};
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONTAINER_OBJECT_PREDICTION_RESULT_H_
