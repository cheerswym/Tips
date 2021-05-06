#ifndef ONBOARD_PLANNER_TEST_UTIL_PREDICTED_TRAJECTORY_BUILDER_H_
#define ONBOARD_PLANNER_TEST_UTIL_PREDICTED_TRAJECTORY_BUILDER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/memory/memory.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace planner {

class PredictedTrajectoryBuilder {
 public:
  PredictedTrajectoryBuilder();

  PredictedTrajectoryBuilder& set_probability(double probability);

  PredictedTrajectoryBuilder& set_priority(ObjectPredictionPriority priority);

  PredictedTrajectoryBuilder& set_annotation(std::string annotation);

  PredictedTrajectoryBuilder& set_type(PredictionType type);

  PredictedTrajectoryBuilder& set_index(int index);

  PredictedTrajectoryBuilder& set_points(
      std::vector<prediction::PredictedTrajectoryPoint> points);

  // Specify a straight line prediction with constant acceleration.
  PredictedTrajectoryBuilder& set_straight_line(Vec2d start, Vec2d end,
                                                double init_v, double last_v);

  PredictedTrajectoryBuilder& set_stationary_traj(Vec2d pos, double theta);

  prediction::PredictedTrajectory Build();

 private:
  double probability_ = 1.0;
  ObjectPredictionPriority priority_ = OPP_P3;
  std::string annotation_ = "obj";
  int index_ = 0;
  PredictionType type_ = PredictionType::PT_CYCV;
  std::vector<prediction::PredictedTrajectoryPoint> points_;
  std::unique_ptr<prediction::PredictedTrajectory> traj_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_PREDICTED_TRAJECTORY_BUILDER_H_
