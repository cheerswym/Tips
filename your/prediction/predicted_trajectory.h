#ifndef ONBOARD_PREDICTION_PREDICTED_TRAJECTORY_H_
#define ONBOARD_PREDICTION_PREDICTED_TRAJECTORY_H_

#include <string>
#include <utility>
#include <vector>

#include "onboard/math/geometry/util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {

class PredictedTrajectoryPoint : public planner::SecondOrderTrajectoryPoint {
 public:
  PredictedTrajectoryPoint() = default;
  explicit PredictedTrajectoryPoint(
      const PredictedTrajectoryPointProto &proto) {
    FromProto(proto);
  }
  // Create from a second-order trajectory point where bmv info is lost.
  explicit PredictedTrajectoryPoint(
      const planner::SecondOrderTrajectoryPoint &point)
      : planner::SecondOrderTrajectoryPoint(point) {}

  // Serialization to proto.
  void FromProto(const PredictedTrajectoryPointProto &proto);
  void ToProto(PredictedTrajectoryPointProto *proto) const;
};

class PredictedTrajectory {
 public:
  PredictedTrajectory() {}
  explicit PredictedTrajectory(const PredictedTrajectoryProto &proto) {
    FromProto(proto);
  }
  PredictedTrajectory(double probability, ObjectPredictionPriority priority,
                      std::string annotation, PredictionType type, int index,
                      std::vector<PredictedTrajectoryPoint> points,
                      bool is_reversed)
      : probability_(probability),
        priority_(priority),
        annotation_(std::move(annotation)),
        type_(type),
        index_(index),
        points_(std::move(points)),
        is_reversed_(is_reversed) {}

  double probability() const { return probability_; }
  ObjectPredictionPriority priority() const { return priority_; }
  const std::string &annotation() const { return annotation_; }
  PredictionType type() const { return type_; }
  bool is_reversed() const { return is_reversed_; }
  int index() const { return index_; }
  const std::vector<PredictedTrajectoryPoint> &points() const {
    return points_;
  }

  void set_probability(double probability) { probability_ = probability; }
  void set_priority(ObjectPredictionPriority priority) { priority_ = priority; }
  void set_annotation(std::string annotation) {
    annotation_ = std::move(annotation);
  }
  void set_type(PredictionType type) { type_ = type; }
  void set_index(int index) { index_ = index; }
  std::vector<PredictedTrajectoryPoint> *mutable_points() { return &points_; }

  Polygon2d CreateContourForPoint(const Polygon2d &obj_contour, int i) const;

  void PrintDebugInfo() const;

  // Serialization to proto.
  void FromProto(const PredictedTrajectoryProto &proto);
  void ToProto(PredictedTrajectoryProto *proto) const;

 private:
  double probability_ = 0.0;
  ObjectPredictionPriority priority_ = OPP_P3;
  std::string annotation_;
  PredictionType type_;
  int index_;
  std::vector<PredictedTrajectoryPoint> points_;
  bool is_reversed_ = false;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTED_TRAJECTORY_H_
