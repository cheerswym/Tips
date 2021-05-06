#ifndef ONBOARD_PLANNER_TEST_UTIL_OBJECT_PREDICTION_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_OBJECT_PREDICTION_BUILDER_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/prediction.h"

namespace qcraft {
namespace planner {

class ObjectPredictionBuilder {
 public:
  ObjectPredictionBuilder() {}

  ObjectPredictionBuilder& set_object(const ObjectProto& object);

  bool HasObject() const { return perception_object_.has_value(); }

  PredictedTrajectoryBuilder* add_predicted_trajectory();

  prediction::ObjectPrediction Build();

 private:
  std::vector<prediction::PredictedTrajectory> trajs_;
  std::optional<ObjectProto> perception_object_;
  std::vector<std::unique_ptr<PredictedTrajectoryBuilder>>
      predicted_trajectory_builders_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_OBJECT_PREDICTION_BUILDER_H_
