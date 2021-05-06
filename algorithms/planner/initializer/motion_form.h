#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_FORM_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_FORM_H_

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/motion_state.h"

namespace qcraft::planner {

enum MotionFormType {
  CONST_ACCEL_MOTION = 1,
  STATIONARY_MOTION = 2,
  EXPERT_COMPLETE_MOTION = 3,
  CANDIDATE_COMPLETE_MOTION = 4,
};

// This represents a short trajectory.
class MotionForm {
 public:
  // The time duration of the motion.
  virtual double duration() const = 0;
  virtual MotionState GetStartMotionState() const = 0;
  virtual MotionState GetEndMotionState() const = 0;
  // Samples the spacetime states from a motion prime at constant time interval.
  virtual std::vector<MotionState> FastSample(double d_t) const = 0;
  virtual std::vector<MotionState> Sample(double d_t) const = 0;
  virtual MotionState State(double t) const = 0;
  virtual const GeometryForm* geometry() const = 0;
  virtual MotionFormType type() const = 0;

  virtual ~MotionForm() {}
};

class ConstAccelMotion final : public MotionForm {
 public:
  // Create by v and a
  ConstAccelMotion(double init_v, double init_a, const GeometryForm* geometry);
  // Create by v0 and v1. Use pair to differentiate constructors' signatures.
  ConstAccelMotion(std::pair<double, double> v_pair,
                   const GeometryForm* geometry);

  double duration() const override { return duration_; }
  MotionState GetStartMotionState() const override;
  MotionState GetEndMotionState() const override;
  // Use fast state of geometry graph to accelerate computation, error 1e-1 m.
  std::vector<MotionState> FastSample(double d_t) const override;
  std::vector<MotionState> Sample(double d_t) const override;

  MotionState State(double t) const override;

  const GeometryForm* geometry() const override { return geometry_; }
  MotionFormType type() const override {
    return MotionFormType::CONST_ACCEL_MOTION;
  }

 private:
  double init_v_ = 0.0;
  double a_ = 0.0;
  double duration_ = 0.0;
  // Time of which the motion stops before the end of geometry. Set to max
  // if sdc still moving at the end node.
  double stop_time_ = 0.0;
  double stop_distance_ = 0.0;
  // Not owned.
  const GeometryForm* geometry_ = nullptr;
};

class StationaryMotion final : public MotionForm {
 public:
  explicit StationaryMotion(double duration, const StationaryGeometry* geometry)
      : duration_(duration), geometry_(geometry) {}

  explicit StationaryMotion(double duration, GeometryState state)
      : duration_(duration),
        geometry_form_(std::make_unique<StationaryGeometry>(std::move(state))),
        geometry_(geometry_form_.get()) {}

  std::vector<MotionState> FastSample(double d_t) const override;
  std::vector<MotionState> Sample(double d_t) const override;

  double duration() const override { return duration_; }
  MotionState GetStartMotionState() const override;
  MotionState GetEndMotionState() const override;
  MotionState State(double t) const override;

  const GeometryForm* geometry() const override { return geometry_; }
  MotionFormType type() const override {
    return MotionFormType::STATIONARY_MOTION;
  }

 private:
  double duration_ = 0.0;
  // Optional: maybe null.
  std::unique_ptr<StationaryGeometry> geometry_form_;

  // Maybe not owned.
  const GeometryForm* geometry_ = nullptr;
};

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_FORM_H_
