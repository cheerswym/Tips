#ifndef ONBOARD_PREDICTION_PREDICTION_H_
#define ONBOARD_PREDICTION_PREDICTION_H_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/math/geometry/util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {

struct ObjectLongTermBehavior {
  double avg_speed;
  double obs_duration = 0.0;

  void FromProto(const ObjectLongTermBehaviorProto &proto) {
    avg_speed = proto.average_speed();
    obs_duration = proto.observation_duration();
  }

  void ToProto(ObjectLongTermBehaviorProto *proto) const {
    proto->set_average_speed(avg_speed);
    proto->set_observation_duration(obs_duration);
  }
};

class ObjectPrediction {
 public:
  ObjectPrediction() = default;

  explicit ObjectPrediction(const ObjectProto &object_proto)
      : perception_object_(object_proto) {}

  ObjectPrediction(std::vector<PredictedTrajectory> trajectories,
                   const ObjectProto &object);
  // Construct without semantic_map, no rebuilding lane_path.
  explicit ObjectPrediction(const ObjectPredictionProto &proto);

  const ObjectProto &perception_object() const { return perception_object_; }

  double timestamp() const { return perception_object_.timestamp(); }

  const std::string &id() const { return perception_object_.id(); }
  const std::vector<PredictedTrajectory> &trajectories() const {
    return trajectories_;
  }
  double trajectory_prob_sum() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 0.0,
                           [](const double sum, const auto &traj) {
                             return sum + traj.probability();
                           });
  }
  double trajectory_max_prob() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 0.0,
                           [](const double max, const auto &traj) {
                             return std::max(max, traj.probability());
                           });
  }
  double trajectory_min_prob() const {
    return std::accumulate(trajectories_.begin(), trajectories_.end(), 1.0,
                           [](const double min, const auto &traj) {
                             return std::min(min, traj.probability());
                           });
  }

  const Polygon2d &contour() const { return contour_; }

  void set_id(std::string id) { perception_object_.set_id(id); }

  std::vector<PredictedTrajectory> *mutable_trajectories() {
    return &trajectories_;
  }

  void CheckTrajectoryProb(double sum = 1.0, double max = 1.0,
                           double min = 0.0) const;
  void PrintDebugInfo() const;

  Polygon2d CreateContourForPoint(int predicted_trajectory_index,
                                  int predicted_point_index) const;
  const ObjectStopTimeProto &stop_time() const { return stop_time_; }
  const ObjectLongTermBehavior &long_term_behavior() const {
    return long_term_behavior_;
  }

  // Serialization to proto.
  void FromProto(const ObjectPredictionProto &proto);
  void ToProto(ObjectPredictionProto *proto) const;

 private:
  std::vector<PredictedTrajectory> trajectories_;
  Polygon2d contour_;

  ObjectProto perception_object_;
  // stop time
  ObjectStopTimeProto stop_time_;

  // Some statictics for object's long-term behavior.
  ObjectLongTermBehavior long_term_behavior_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTION_H_
