#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AV_MODEL_HELPER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AV_MODEL_HELPER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/cost_helper.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class AvModelHelper : public CostHelper<PROB> {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  AvModelHelper(double dists_to_rac, double angles_to_axis,
                bool enable_fast_math, std::string name)
      : CostHelper<PROB>(std::move(name)),
        dist_to_rac_(dists_to_rac),
        angle_to_axis_(angles_to_axis),
        enable_fast_math_(enable_fast_math) {
    centers_.resize(PROB::kHorizon);
    tangents_.resize(PROB::kHorizon);
    normals_.resize(PROB::kHorizon);
  }

  const std::vector<Vec2d> &centers() const { return centers_; }
  const std::vector<Vec2d> &tangents() const { return tangents_; }
  const std::vector<Vec2d> &normals() const { return normals_; }

  double dist_to_rac() const { return dist_to_rac_; }
  double angle_to_axis() const { return angle_to_axis_; }

  void Update(const StatesType &xs, const ControlsType &us) override {
    for (int k = 0; k < PROB::kHorizon; ++k) {
      const Vec2d pos = PROB::pos(xs, k);
      const Vec2d tangent =
          enable_fast_math_
              ? Vec2d::FastUnitFromAngle(PROB::theta(xs, k) + angle_to_axis_)
              : Vec2d::UnitFromAngle(PROB::theta(xs, k) + angle_to_axis_);
      tangents_[k] = tangent;
      normals_[k] = tangent.Perp();
      centers_[k] = pos + tangent * dist_to_rac_;
    }
  }

 private:
  std::string name_ = "";
  double dist_to_rac_;
  double angle_to_axis_;
  int circle_num_;
  bool enable_fast_math_;

  std::vector<Vec2d> centers_;
  std::vector<Vec2d> tangents_;
  std::vector<Vec2d> normals_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AV_MODEL_HELPER_H_
