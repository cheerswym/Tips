#ifndef ONBOARD_PLANNER_INITIALIZER_CANDIDATE_COMPLETE_MOTION_FORM_H_
#define ONBOARD_PLANNER_INITIALIZER_CANDIDATE_COMPLETE_MOTION_FORM_H_

#include <limits>
#include <vector>

#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"

namespace qcraft::planner {

class CandidateCompleteMotion final : public MotionForm {
 public:
  explicit CandidateCompleteMotion(
      const GeometryFormBuilder* form_builder,
      const std::vector<ApolloTrajectoryPointProto>& candidate_traj);

  std::vector<MotionState> FastSample(double d_t) const override;
  std::vector<MotionState> Sample(double d_t) const override;

  double duration() const override { return duration_; }
  MotionState GetStartMotionState() const override;
  MotionState GetEndMotionState() const override;

  /**
  @brief: Use relative time to get a motion state
  **/
  MotionState State(double t) const override;

  const GeometryForm* geometry() const override { return nullptr; }

  MotionFormType type() const override {
    return MotionFormType::CANDIDATE_COMPLETE_MOTION;
  }

 private:
  double duration_;
  std::vector<ApolloTrajectoryPointProto> candidate_traj_;
  std::vector<double> relative_stations_;
  const GeometryFormBuilder* form_builder_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_CANDIDATE_COMPLETE_MOTION_FORM_H_
