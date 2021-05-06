#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_S_LIMITING_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_S_LIMITING_OBSTACLE_COST_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/lite/logging.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class TobSLimitingObjectCost : public Cost<PROB> {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  using DividedG = typename Cost<PROB>::DividedG;

  // penetration =
  // (s + standoff - s_ref) if type = Backward;
  // (s_ref - s + standoff) if type = Forward.
  // Cost = 0.5 * scale * penetration^2,      if penetration > 0;
  //        0,                                if penetration <= 0.
  struct Object {
    // Reference point on reference path.
    std::string id;
    Vec2d ref = Vec2d::Zero();
    // Heading direction of refrence point.
    Vec2d dir = Vec2d::Zero();
    // Reference s on reference path.
    double s_ref = 0.0;
    // Individual object can have a variable gain on top of the uniform scale.
    double gain = 1.0;
    // Max penetration before deactivating.
    double depth = 10.0;  // m.
    double standoff = 0.0;
    enum class Type {
      kForward,   // Push AV forwards
      kBackward,  // Push AV backwards
    };
    Type type = Type::kBackward;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  TobSLimitingObjectCost(
      const VehicleGeometryParamsProto *vehicle_geometry_params,
      std::array<Object, PROB::kHorizon> objects,
      std::string name = "TobSLimitingObjectCost", double scale = 1.0)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        objects_(std::move(objects)) {
    penetrations_.fill(-std::numeric_limits<double>::infinity());
    penetrations_dir_.fill(0.0);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      standoffs_[k] = objects_[k].standoff;
    }
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (penetrations_[k] > 0.0) {
        res.AddSubG(
            /*idx=*/0, 0.5 * Cost<PROB>::scale() * objects_[k].gain *
                           Sqr(penetrations_[k]));
      }
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    double g = 0.0;
    if (penetrations_[k] > 0.0) {
      g += 0.5 * Cost<PROB>::scale() * objects_[k].gain * Sqr(penetrations_[k]);
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    if (penetrations_[k] > 0.0) {
      (*dgdx)[PROB::kStateSIndex] += Cost<PROB>::scale() * objects_[k].gain *
                                     penetrations_[k] * penetrations_dir_[k];
    }
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    if (penetrations_[k] > 0.0) {
      (*ddgdxdx)(PROB::kStateSIndex, PROB::kStateSIndex) +=
          Cost<PROB>::scale() * objects_[k].gain;
    }
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}
  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    const double rac2fb = vehicle_geometry_params_.front_edge_to_center();
    const double rac2rb = vehicle_geometry_params_.back_edge_to_center();
    for (int k = 0; k < PROB::kHorizon; ++k) {
      const Object &ob = objects_[k];

      if (!ob.enable) continue;
      const auto x = PROB::GetStateMapAtStep(xs, k);
      VLOG(4) << "Step " << k;
      VLOG(4) << "x = " << x.transpose();

      switch (ob.type) {
        case Object::Type::kBackward: {
          const double s = PROB::StateGetS(x);
          penetrations_[k] = s + rac2fb + standoffs_[k] - ob.s_ref;
          penetrations_dir_[k] = 1.0;
          VLOG(4) << "At step " << k << " for Backward ob " << ob.id
                  << ": type = " << static_cast<int>(ob.type) << " s = " << s
                  << " rac2fb = " << rac2fb << " standoff = " << standoffs_[k]
                  << " ob.s_ref " << ob.s_ref
                  << " penetration = " << penetrations_[k];
          break;
        }
        case Object::Type::kForward: {
          const double s = PROB::StateGetS(x);
          penetrations_[k] = ob.s_ref - s + rac2rb + standoffs_[k];
          penetrations_dir_[k] = -1.0;
          VLOG(4) << "At step " << k << " for Forward ob " << ob.id
                  << ": type = " << static_cast<int>(ob.type) << " s = " << s
                  << " rac2fb = " << rac2fb << " standoff = " << standoffs_[k]
                  << " ob.s_ref " << ob.s_ref
                  << " penetration = " << penetrations_[k];
          break;
        }
        default:
          QLOG(FATAL) << "Unknown object type " << static_cast<int>(ob.type);
          break;
      }
      // Restrict penetration to object depth to prevent overreaction.
      penetrations_[k] = std::min(penetrations_[k], ob.depth);
    }
  }

 private:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::array<Object, PROB::kHorizon> objects_;

  // States.
  std::array<double, PROB::kHorizon> standoffs_;
  std::array<double, PROB::kHorizon> penetrations_;
  std::array<double, PROB::kHorizon> penetrations_dir_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_S_LIMITING_OBSTACLE_COST_H_
