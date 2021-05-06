#ifndef ONBOARD_PLANNER_INITIALIZER_EXPERT_COMPLETE_MOTION_FORM_H_
#define ONBOARD_PLANNER_INITIALIZER_EXPERT_COMPLETE_MOTION_FORM_H_

#include <limits>
#include <vector>

#include "offboard/planner/ml/datasets/pnc_scenario_dataset/proto/pnc_scenario.pb.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/utils/history_buffer.h"

namespace qcraft::planner {

class ExpertCompleteMotion final : public MotionForm {
 public:
  explicit ExpertCompleteMotion(
      const GeometryFormBuilder* form_builder,
      const HistoryBuffer<planning_dataset::ObjectState>& expert_traj);

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
    return MotionFormType::EXPERT_COMPLETE_MOTION;
  }

 private:
  double duration_ = 0.0;
  HistoryBuffer<planning_dataset::ObjectState> expert_traj_;
  std::vector<double> relative_stations_;
  const GeometryFormBuilder* form_builder_;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_EXPERT_COMPLETE_MOTION_FORM_H_
