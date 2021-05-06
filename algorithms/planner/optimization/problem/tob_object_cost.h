#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_OBSTACLE_COST_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/third_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class TobObjectCost : public Cost<PROB> {
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

  // penetration = (x - ref).dot(dir)
  // Cost = 0.5 * scale * penetration^2,      if penetration > 0;
  //        0,                                if penetration <= 0.
  // x could be the AV RAC, front left corner, or front right corner, depending
  // on type.
  struct Object {
    // Direction that the cost increases in.
    Vec2d dir = Vec2d::UnitX();
    // Reference point.
    Vec2d ref = Vec2d::Zero();
    // Lateral extent (full extent is twice this much).
    double lateral_extent = 0.0;
    // Individual object can have a variable gain on top of the uniform scale.
    double gain = 1.0;
    // Max penetration before deactivating.
    double depth = 10.0;  // m.

    enum class Type {
      kRac,
      kFrontLeft,
      kFrontRight,
      kFrontCenter,
    };
    Type type = Type::kRac;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  explicit TobObjectCost(
      const VehicleGeometryParamsProto *vehicle_geometry_params,
      std::array<Object, PROB::kHorizon> objects,
      std::string name = "TobObjectCost", double scale = 1.0,
      bool enable_fast_math = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, enable_fast_math),
        vehicle_geometry_params_(*CHECK_NOTNULL(vehicle_geometry_params)),
        objects_(std::move(objects)) {
    penetrations_.fill(-std::numeric_limits<double>::infinity());
    penetration_jacobians_.fill(PenetrationJacobianType::Zero());
    penetration_hessians_.fill(PenetrationHessianType::Zero());
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(/*size=*/1);
    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (penetrations_[k] > 0.0) {
        res.AddSubG(
            /*idx=*/0, objects_[k].gain * Sqr(penetrations_[k]));
      }
    }
    res.SetSubName(/*idx=*/0, Cost<PROB>::name());
    res.Multi(0.5 * Cost<PROB>::scale());
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
      *dgdx += Cost<PROB>::scale() * objects_[k].gain * penetrations_[k] *
               penetration_jacobians_[k];
    }
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    if (penetrations_[k] > 0.0) {
      *ddgdxdx +=
          Cost<PROB>::scale() * objects_[k].gain *
          (penetrations_[k] * penetration_hessians_[k] +
           penetration_jacobians_[k].transpose() * penetration_jacobians_[k]);
    }
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    const double rac2fb = vehicle_geometry_params_.front_edge_to_center();
    const double half_width = vehicle_geometry_params_.width() * 0.5;

    const auto in_lat_range = [](Vec2d p0, Vec2d p1, const Object &ob) {
      const double lat_offset0 = ob.dir.CrossProd(p0 - ob.ref);
      const double lat_offset1 = ob.dir.CrossProd(p1 - ob.ref);
      if ((lat_offset0 > ob.lateral_extent &&
           lat_offset1 > ob.lateral_extent) ||
          (lat_offset0 < -ob.lateral_extent &&
           lat_offset1 < -ob.lateral_extent)) {
        return false;
      }
      return true;
    };
    for (int k = 0; k < PROB::kHorizon; ++k) {
      penetrations_[k] = -std::numeric_limits<double>::infinity();
      penetration_jacobians_[k](PROB::kStateXIndex) = 0.0;
      penetration_jacobians_[k](PROB::kStateYIndex) = 0.0;
      penetration_jacobians_[k](PROB::kStateThetaIndex) = 0.0;
      penetration_hessians_[k](PROB::kStateThetaIndex, PROB::kStateThetaIndex) =
          0.0;
      const Object &ob = objects_[k];
      if (!ob.enable) continue;

      const auto x0 = PROB::GetStateMapAtStep(xs, k);
      const auto x1 =
          PROB::GetStateMapAtStep(xs, std::min(PROB::kHorizon - 1, k + 1));
      const Vec2d av_tangent =
          Cost<PROB>::enable_fast_math()
              ? Vec2d::FastUnitFromAngle(PROB::StateGetTheta(x0))
              : Vec2d::UnitFromAngle(PROB::StateGetTheta(x0));
      const Vec2d av_normal = av_tangent.Perp();
      VLOG(3) << "Step " << k;
      VLOG(4) << "x0 = " << x0.transpose() << " x1 = " << x1.transpose()
              << " av_tangent = " << av_tangent.transpose()
              << " av_normal = " << av_normal.transpose();

      switch (ob.type) {
        case Object::Type::kRac: {
          const Vec2d pos0 = PROB::StateGetPos(x0);
          const Vec2d pos1 = PROB::StateGetPos(x1);
          if (!in_lat_range(pos0, pos1, ob)) break;
          penetrations_[k] = (pos0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " pos0 = " << pos0.transpose()
                  << " pos1 = " << pos1.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
        case Object::Type::kFrontLeft: {
          const Vec2d front_left0 = PROB::StateGetPos(x0) +
                                    av_tangent * rac2fb +
                                    av_normal * half_width;
          const Vec2d front_left1 = PROB::StateGetPos(x1) +
                                    av_tangent * rac2fb +
                                    av_normal * half_width;
          if (!in_lat_range(front_left0, front_left1, ob)) break;
          penetrations_[k] = (front_left0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          penetration_jacobians_[k](PROB::kStateThetaIndex) =
              ob.dir.dot(av_normal * rac2fb - av_tangent * half_width);
          penetration_hessians_[k](PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
              ob.dir.dot(-av_tangent * rac2fb - av_normal * half_width);
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " front_left0 = " << front_left0.transpose()
                  << " front_left1 = " << front_left1.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
        case Object::Type::kFrontRight: {
          const Vec2d front_right0 = PROB::StateGetPos(x0) +
                                     av_tangent * rac2fb -
                                     av_normal * half_width;
          const Vec2d front_right1 = PROB::StateGetPos(x1) +
                                     av_tangent * rac2fb -
                                     av_normal * half_width;
          if (!in_lat_range(front_right0, front_right1, ob)) break;
          penetrations_[k] = (front_right0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          penetration_jacobians_[k](PROB::kStateThetaIndex) =
              ob.dir.dot(av_normal * rac2fb + av_tangent * half_width);
          penetration_hessians_[k](PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
              ob.dir.dot(-av_tangent * rac2fb + av_normal * half_width);
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " front_right0 = " << front_right0.transpose()
                  << " front_right1 = " << front_right1.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
        case Object::Type::kFrontCenter: {
          const Vec2d front_center0 =
              PROB::StateGetPos(x0) + av_tangent * rac2fb;
          const Vec2d front_center1 =
              PROB::StateGetPos(x1) + av_tangent * rac2fb;
          if (!in_lat_range(front_center0, front_center1, ob)) break;
          penetrations_[k] = (front_center0 - ob.ref).dot(ob.dir);
          penetration_jacobians_[k].template segment<2>(PROB::kStateXIndex) =
              ob.dir.transpose();
          penetration_jacobians_[k](PROB::kStateThetaIndex) =
              ob.dir.dot(av_normal * rac2fb);
          penetration_hessians_[k](PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
              ob.dir.dot(-av_tangent * rac2fb);
          VLOG(4) << "  ob: type = " << static_cast<int>(ob.type)
                  << " front_center0 = " << front_center0.transpose()
                  << " front_center1 = " << front_center0.transpose()
                  << " penetration = " << penetrations_[k]
                  << " jacobian = " << penetration_jacobians_[k];
          break;
        }
      }
      // Ignore very deep penetrations; such cases usually correspond to the
      // AV bypassing the object entirely.
      if (penetrations_[k] > ob.depth) penetrations_[k] = 0.0;
    }
  }

 private:
  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::array<Object, PROB::kHorizon> objects_;

  // States.
  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  std::array<double, PROB::kHorizon> penetrations_;
  std::array<PenetrationJacobianType, PROB::kHorizon> penetration_jacobians_;
  std::array<PenetrationHessianType, PROB::kHorizon> penetration_hessians_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_TOB_OBSTACLE_COST_H_
