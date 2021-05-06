#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_STEP_STATIC_BOUNDARY_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_STEP_STATIC_BOUNDARY_COST_H_

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// A static boundary represented by a sequence of path points, and the
// associated boundary distances to those path points. The difference from
// StaticBoundaryCost is that this cost treat distance from boundary to path as
// a step function.
template <typename PROB>
class StepStaticBoundaryCost : public Cost<PROB> {
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

  static constexpr double kNormalizedScale = 10.0;
  StepStaticBoundaryCost(VehicleGeometryParamsProto vehicle_geometry_params,
                         std::vector<Vec2d> path_points,
                         std::vector<double> path_boundary_dists, bool left,
                         std::vector<double> mid_edges_to_center,
                         std::vector<std::string> sub_names,
                         std::vector<double> cascade_buffers = {0.0},
                         std::vector<double> cascade_gains = {1.0},
                         std::string name = absl::StrCat(
                             PROB::kProblemPrefix, "StepStaticBoundaryCost"),
                         bool ignore_head = false, bool ignore_tail = false,
                         double scale = 1.0, bool enable_fast_math = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, enable_fast_math),
        vehicle_geometry_params_(std::move(vehicle_geometry_params)),
        path_points_(std::move(path_points)),
        path_boundary_dists_(std::move(path_boundary_dists)),
        left_(left),
        mid_edges_to_center_(std::move(mid_edges_to_center)),
        cascade_buffers_(std::move(cascade_buffers)),
        cascade_gains_(std::move(cascade_gains)),
        sub_names_(std::move(sub_names)),
        ignore_head_(ignore_head),
        ignore_tail_(ignore_tail) {
    QCHECK_GT(path_points_.size(), 1);
    QCHECK_EQ(path_points_.size(), path_boundary_dists_.size());
    path_ = std::make_unique<KdTreeFrenetFrame>(
        BuildKdTreeFrenetFrame(path_points_).value());
    mid_normals_.resize(mid_edges_to_center_.size());
    mid_dists_.resize(mid_edges_to_center_.size());
    mid_boundary_dists_.resize(mid_edges_to_center_.size());
    mid_index_pairs_.resize(mid_edges_to_center_.size());
    mid_alphas_.resize(mid_edges_to_center_.size());
    mid_beyond_head_.resize(mid_edges_to_center_.size());
    mid_beyond_tail_.resize(mid_edges_to_center_.size());
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(sub_names_.size());
    const double half_width = vehicle_geometry_params_.width() * 0.5;
    for (int k = 0; k < PROB::kHorizon; ++k) {
      // Cost for rac point
      if (ignore_head_ && state_beyond_head_[k]) {  // do not compute cost
      } else if (ignore_tail_ &&
                 state_beyond_tail_[k]) {  // do not compute cost
      } else {
        const double d =
            left_ ? state_boundary_dists_[k] - state_dists_[k] - half_width
                  : state_boundary_dists_[k] + state_dists_[k] - half_width;
        for (int i = 0; i < cascade_buffers_.size(); ++i) {
          if (d < cascade_buffers_[i]) {
            res.AddSubG(i, 0.5 * Cost<PROB>::scale() * cascade_gains_[i] *
                               Sqr(cascade_buffers_[i] - d));
          }
        }
      }

      // Cost for corner point
      if (ignore_head_ && corner_beyond_head_[k]) {  // do not compute cost
      } else if (ignore_tail_ &&
                 corner_beyond_tail_[k]) {  // do not compute cost
      } else {
        const double d_corner =
            left_ ? corner_boundary_dists_[k] - corner_dists_[k]
                  : corner_boundary_dists_[k] + corner_dists_[k];
        for (int i = 0; i < cascade_buffers_.size(); ++i) {
          if (d_corner < cascade_buffers_[i]) {
            res.AddSubG(i, 0.5 * Cost<PROB>::scale() * cascade_gains_[i] *
                               Sqr(cascade_buffers_[i] - d_corner));
          }
        }
      }

      // Cost for mid edge point
      for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
        if (ignore_head_ && mid_beyond_head_[i][k]) {  // do not compute cost
        } else if (ignore_tail_ &&
                   mid_beyond_tail_[i][k]) {  // do not compute cost
        } else {
          const double d_mid =
              left_ ? mid_boundary_dists_[i][k] - mid_dists_[i][k]
                    : mid_boundary_dists_[i][k] + mid_dists_[i][k];
          for (int j = 0; j < cascade_buffers_.size(); ++j) {
            if (d_mid < cascade_buffers_[j]) {
              res.AddSubG(i, 0.5 * Cost<PROB>::scale() * cascade_gains_[j] *
                                 Sqr(cascade_buffers_[j] - d_mid));
            }
          }
        }
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, Cost<PROB>::name() + sub_names_[i]);
    }
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType &x,
                                  const ControlType &u,
                                  bool using_scale) const override {
    double scale;
    if (using_scale) {
      scale = Cost<PROB>::scale();
    } else {
      scale = 1.0;
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, Cost<PROB>::name() + sub_names_[i]);
    }
    const double half_width = vehicle_geometry_params_.width() * 0.5;

    // Cost for rac point
    if (ignore_head_ && state_beyond_head_[k]) {         // do not compute cost
    } else if (ignore_tail_ && state_beyond_tail_[k]) {  // do not compute cost
    } else {
      const double d =
          left_ ? state_boundary_dists_[k] - state_dists_[k] - half_width
                : state_boundary_dists_[k] + state_dists_[k] - half_width;
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d < cascade_buffers_[i]) {
          res.AddSubG(i, 0.5 * scale * cascade_gains_[i] *
                             Sqr(cascade_buffers_[i] - d));
        }
      }
    }

    // Cost for corner point
    if (ignore_head_ && corner_beyond_head_[k]) {         // do not compute cost
    } else if (ignore_tail_ && corner_beyond_tail_[k]) {  // do not compute cost
    } else {
      const double d_corner =
          left_ ? corner_boundary_dists_[k] - corner_dists_[k]
                : corner_boundary_dists_[k] + corner_dists_[k];
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d_corner < cascade_buffers_[i]) {
          res.AddSubG(i, 0.5 * scale * cascade_gains_[i] *
                             Sqr(cascade_buffers_[i] - d_corner));
        }
      }
    }

    // Cost for mid edge point
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      if (ignore_head_ && mid_beyond_head_[i][k]) {  // do not compute cost
      } else if (ignore_tail_ &&
                 mid_beyond_tail_[i][k]) {  // do not compute cost
      } else {
        const double d_mid = left_
                                 ? mid_boundary_dists_[i][k] - mid_dists_[i][k]
                                 : mid_boundary_dists_[i][k] + mid_dists_[i][k];
        for (int j = 0; j < cascade_buffers_.size(); ++j) {
          if (d_mid < cascade_buffers_[j]) {
            res.AddSubG(i, 0.5 * scale * cascade_gains_[j] *
                               Sqr(cascade_buffers_[j] - d_mid));
          }
        }
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    const double half_width = vehicle_geometry_params_.width() * 0.5;

    double g = 0.0;
    // Cost for rac point
    if (ignore_head_ && state_beyond_head_[k]) {         // do not compute cost
    } else if (ignore_tail_ && state_beyond_tail_[k]) {  // do not compute cost
    } else {
      const double d =
          left_ ? state_boundary_dists_[k] - state_dists_[k] - half_width
                : state_boundary_dists_[k] + state_dists_[k] - half_width;
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d < cascade_buffers_[i]) {
          g += 0.5 * Cost<PROB>::scale() * cascade_gains_[i] *
               Sqr(cascade_buffers_[i] - d);
        }
      }
    }

    // Cost for corner point
    if (ignore_head_ && corner_beyond_head_[k]) {         // do not compute cost
    } else if (ignore_tail_ && corner_beyond_tail_[k]) {  // do not compute cost
    } else {
      const double d_corner =
          left_ ? corner_boundary_dists_[k] - corner_dists_[k]
                : corner_boundary_dists_[k] + corner_dists_[k];
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d_corner < cascade_buffers_[i]) {
          g += 0.5 * Cost<PROB>::scale() * cascade_gains_[i] *
               Sqr(cascade_buffers_[i] - d_corner);
        }
      }
    }

    // Cost for mid edge point
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      if (ignore_head_ && mid_beyond_head_[i][k]) {  // do not compute cost
      } else if (ignore_tail_ &&
                 mid_beyond_tail_[i][k]) {  // do not compute cost
      } else {
        const double d_mid = left_
                                 ? mid_boundary_dists_[i][k] - mid_dists_[i][k]
                                 : mid_boundary_dists_[i][k] + mid_dists_[i][k];
        for (int j = 0; j < cascade_buffers_.size(); ++j) {
          if (d_mid < cascade_buffers_[j]) {
            g += 0.5 * Cost<PROB>::scale() * cascade_gains_[j] *
                 Sqr(cascade_buffers_[j] - d_mid);
          }
        }
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    const double theta = PROB::StateGetTheta(x);
    const double half_width = vehicle_geometry_params_.width() * 0.5;
    const double cos_theta = Cost<PROB>::enable_fast_math()
                                 ? fast_math::Cos(theta)
                                 : std::cos(theta);
    const double sin_theta = Cost<PROB>::enable_fast_math()
                                 ? fast_math::Sin(theta)
                                 : std::sin(theta);
    // Gradient for rac point
    if (ignore_head_ && state_beyond_head_[k]) {  // do not compute gradient
    } else if (ignore_tail_ &&
               state_beyond_tail_[k]) {  // do not compute gradient
    } else {
      const double d =
          left_ ? state_boundary_dists_[k] - state_dists_[k] - half_width
                : state_boundary_dists_[k] + state_dists_[k] - half_width;
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d < cascade_buffers_[i]) {
          const double d_penetration = d - cascade_buffers_[i];
          const double sign = left_ ? -1.0 : 1.0;
          (*dgdx).template segment<2>(PROB::kStateXIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] * d_penetration * sign *
              state_normals_[k].transpose();
        }
      }
    }

    // Gradient for corner point
    AddEdgePointDGDx(dgdx, corner_boundary_dists_[k], corner_dists_[k],
                     vehicle_geometry_params_.front_edge_to_center(), sin_theta,
                     cos_theta, half_width, corner_beyond_head_[k],
                     corner_beyond_tail_[k], corner_index_pairs_[k],
                     corner_alphas_[k], corner_normals_[k]);

    // Gradient for mid edge point
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      AddEdgePointDGDx(dgdx, mid_boundary_dists_[i][k], mid_dists_[i][k],
                       mid_edges_to_center_[i], sin_theta, cos_theta,
                       half_width, mid_beyond_head_[i][k],
                       mid_beyond_tail_[i][k], mid_index_pairs_[i][k],
                       mid_alphas_[i][k], mid_normals_[i][k]);
    }
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    const double theta = PROB::StateGetTheta(x);
    const double half_width = vehicle_geometry_params_.width() * 0.5;
    const double cos_theta = Cost<PROB>::enable_fast_math()
                                 ? fast_math::Cos(theta)
                                 : std::cos(theta);
    const double sin_theta = Cost<PROB>::enable_fast_math()
                                 ? fast_math::Sin(theta)
                                 : std::sin(theta);
    // Hessian for rac point
    if (ignore_head_ && state_beyond_head_[k]) {  // do not compute Hessian
    } else if (ignore_tail_ &&
               state_beyond_tail_[k]) {  // do not compute Hessian
    } else {
      const double d =
          left_ ? state_boundary_dists_[k] - state_dists_[k] - half_width
                : state_boundary_dists_[k] + state_dists_[k] - half_width;
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d < cascade_buffers_[i]) {
          (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                          PROB::kStateXIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] * state_normals_[k] *
              state_normals_[k].transpose();
        }
      }
    }

    // Hessian for corner point.
    AddEdgePointDDGDxDx(
        ddgdxdx, corner_boundary_dists_[k], corner_dists_[k],
        vehicle_geometry_params_.front_edge_to_center(), sin_theta, cos_theta,
        half_width, corner_beyond_head_[k], corner_beyond_tail_[k],
        corner_index_pairs_[k], corner_alphas_[k], corner_normals_[k]);

    // Hessian for mid point.
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      AddEdgePointDDGDxDx(ddgdxdx, mid_boundary_dists_[i][k], mid_dists_[i][k],
                          mid_edges_to_center_[i], sin_theta, cos_theta,
                          half_width, mid_beyond_head_[i][k],
                          mid_beyond_tail_[i][k], mid_index_pairs_[i][k],
                          mid_alphas_[i][k], mid_normals_[i][k]);
    }
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    if (left_) {
      VLOG(4) << "Update rac for left boundary.";
    } else {
      VLOG(4) << "Update rac for right boundary.";
    }
    // Update normal and dist of rac at each state step
    Update(GetStatePosAtStep, xs, &state_normals_, &state_dists_,
           &state_boundary_dists_, &state_index_pairs_, &state_alphas_,
           &state_beyond_head_, &state_beyond_tail_);
    if (left_) {
      VLOG(4) << "Update corner for left boundary.";
    } else {
      VLOG(4) << "Update corner for right boundary.";
    }
    // Update normal and dist of corner at each state step
    Update(std::bind(GetEdgePointPosAtStep, std::placeholders::_1,
                     std::placeholders::_2,
                     vehicle_geometry_params_.front_edge_to_center(),
                     vehicle_geometry_params_, left_,
                     Cost<PROB>::enable_fast_math()),
           xs, &corner_normals_, &corner_dists_, &corner_boundary_dists_,
           &corner_index_pairs_, &corner_alphas_, &corner_beyond_head_,
           &corner_beyond_tail_);
    if (left_) {
      VLOG(4) << "Update mid edge for left boundary.";
    } else {
      VLOG(4) << "Update mid edge for right boundary.";
    }
    // Update normal and dist of mid edge at each state step
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      Update(std::bind(GetEdgePointPosAtStep, std::placeholders::_1,
                       std::placeholders::_2, mid_edges_to_center_[i],
                       vehicle_geometry_params_, left_,
                       Cost<PROB>::enable_fast_math()),
             xs, &mid_normals_[i], &mid_dists_[i], &mid_boundary_dists_[i],
             &mid_index_pairs_[i], &mid_alphas_[i], &mid_beyond_head_[i],
             &mid_beyond_tail_[i]);
    }
  }

 private:
  static constexpr double kBoundaryMatchAlphaThreshold = 0.1;
  static Vec2d GetStatePosAtStep(const StatesType &xs, int k) {
    return PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
  }
  static Vec2d GetEdgePointPosAtStep(
      const StatesType &xs, int k, double dist_to_rac,
      const VehicleGeometryParamsProto &vehicle_geometry_params, bool left,
      bool enable_fast_math) {
    // If the boundary is on the left, corner is AV left front; if the
    // boundary is on the right, corner is right front
    const StateType state = PROB::GetStateAtStep(xs, k);
    const double x = PROB::StateGetX(state);
    const double y = PROB::StateGetY(state);
    const double theta = PROB::StateGetTheta(state);
    const double half_width = vehicle_geometry_params.width() * 0.5;
    const double cos_theta =
        enable_fast_math ? fast_math::Cos(theta) : std::cos(theta);
    const double sin_theta =
        enable_fast_math ? fast_math::Sin(theta) : std::sin(theta);
    return left ? Vec2d(x + dist_to_rac * cos_theta - half_width * sin_theta,
                        y + dist_to_rac * sin_theta + half_width * cos_theta)
                : Vec2d(x + dist_to_rac * cos_theta + half_width * sin_theta,
                        y + dist_to_rac * sin_theta - half_width * cos_theta);
  }

  void Update(
      const std::function<Vec2d(const StatesType &, int)> &get_pos_at_step,
      const StatesType &xs, std::array<Vec2d, PROB::kHorizon> *normals,
      std::array<double, PROB::kHorizon> *dists,
      std::array<double, PROB::kHorizon> *boundary_dists,
      std::array<std::pair<int, int>, PROB::kHorizon> *index_pairs,
      std::array<double, PROB::kHorizon> *alphas,
      std::array<bool, PROB::kHorizon> *beyond_head,
      std::array<bool, PROB::kHorizon> *beyond_tail) const {
    for (int k = 0; k < PROB::kHorizon; ++k) {
      const Vec2d xy = get_pos_at_step(xs, k);
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;
      path_->XYToSL(xy, &sl, &normal, &index_pair, &alpha);
      (*normals)[k] = normal;
      (*dists)[k] = sl.l;
      (*index_pairs)[k] = index_pair;
      (*alphas)[k] = alpha;
      (*boundary_dists)[k] = alpha > kBoundaryMatchAlphaThreshold
                                 ? path_boundary_dists_[index_pair.second]
                                 : path_boundary_dists_[index_pair.first];
      (*beyond_head)[k] = (index_pair.first == 0 && alpha < 0.0);
      (*beyond_tail)[k] =
          (index_pair.second == path_points_.size() - 1 && alpha > 1.0);
      if (UNLIKELY(VLOG_IS_ON(4))) {
        VLOG(4) << "Dists [" << k << "]: " << dists->at(k);
        VLOG(4) << "Normal [" << k << "]: " << normals->at(k).x() << " "
                << normals->at(k).y();
        VLOG(4) << "Index pair [" << k << "]: " << index_pairs->at(k).first
                << " " << index_pairs->at(k).second;
        VLOG(4) << "Alpha [" << k << "]: " << alphas->at(k);
        VLOG(4) << "Boundary dists [" << k << "]: " << boundary_dists->at(k);
        VLOG(4) << "Beyond head [" << k << "]: " << beyond_head->at(k);
        VLOG(4) << "Beyond tail [" << k << "]: " << beyond_tail->at(k);
      }
    }
  }

  void AddEdgePointDGDx(DGDxType *dgdx, double corner_boundary_dist,
                        double corner_dist, double dist_to_rac,
                        double sin_theta, double cos_theta, double half_width,
                        bool corner_beyond_head, bool corner_beyond_tail,
                        const std::pair<int, int> &corner_index_pair,
                        double corner_alpha, const Vec2d &corner_normal) const {
    if (ignore_head_ && corner_beyond_head) {         // do not compute gradient
    } else if (ignore_tail_ && corner_beyond_tail) {  // do not compute gradient
    } else {
      const double d_corner = left_ ? corner_boundary_dist - corner_dist
                                    : corner_boundary_dist + corner_dist;
      Vec2d d_delta;
      if (!left_) {
        d_delta[0] = -dist_to_rac * sin_theta + half_width * cos_theta;
        d_delta[1] = dist_to_rac * cos_theta + half_width * sin_theta;
      } else {
        d_delta[0] = -dist_to_rac * sin_theta - half_width * cos_theta;
        d_delta[1] = dist_to_rac * cos_theta - half_width * sin_theta;
      }
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d_corner < cascade_buffers_[i]) {
          const double d_penetration = d_corner - cascade_buffers_[i];
          const double sign = left_ ? -1.0 : 1.0;
          (*dgdx).template segment<2>(PROB::kStateXIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] * d_penetration * sign *
              corner_normal.transpose();
          (*dgdx)[PROB::kStateThetaIndex] += Cost<PROB>::scale() *
                                             cascade_gains_[i] * d_penetration *
                                             sign * corner_normal.dot(d_delta);
        }
      }
    }
  }
  void AddEdgePointDDGDxDx(DDGDxDxType *ddgdxdx, double corner_boundary_dist,
                           double corner_dist, double dist_to_rac,
                           double sin_theta, double cos_theta,
                           double half_width, bool corner_beyond_head,
                           bool corner_beyond_tail,
                           const std::pair<int, int> &corner_index_pair,
                           double corner_alpha,
                           const Vec2d &corner_normal) const {
    if (ignore_head_ && corner_beyond_head) {         // do not compute Hessian
    } else if (ignore_tail_ && corner_beyond_tail) {  // do not compute Hessian
    } else {
      const double d_corner = left_ ? corner_boundary_dist - corner_dist
                                    : corner_boundary_dist + corner_dist;
      Vec2d d_delta, dd_delta;
      if (!left_) {
        d_delta[0] = -dist_to_rac * sin_theta + half_width * cos_theta;
        d_delta[1] = dist_to_rac * cos_theta + half_width * sin_theta;
        dd_delta[0] = -dist_to_rac * cos_theta - half_width * sin_theta;
        dd_delta[1] = -dist_to_rac * sin_theta + half_width * cos_theta;
      } else {
        d_delta[0] = -dist_to_rac * sin_theta - half_width * cos_theta;
        d_delta[1] = dist_to_rac * cos_theta - half_width * sin_theta;
        dd_delta[0] = -dist_to_rac * cos_theta + half_width * sin_theta;
        dd_delta[1] = -dist_to_rac * sin_theta - half_width * cos_theta;
      }
      for (int i = 0; i < cascade_buffers_.size(); ++i) {
        if (d_corner < cascade_buffers_[i]) {
          const double d_penetration = d_corner - cascade_buffers_[i];
          const double sign = left_ ? -1.0 : 1.0;
          (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                          PROB::kStateXIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] * corner_normal *
              corner_normal.transpose();
          (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                          PROB::kStateThetaIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] * corner_normal *
              corner_normal.dot(d_delta);
          (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                          PROB::kStateXIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] *
              corner_normal.transpose() * corner_normal.dot(d_delta);
          (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
              Cost<PROB>::scale() * cascade_gains_[i] *
              (Sqr(corner_normal.dot(d_delta)) +
               d_penetration * sign * corner_normal.dot(dd_delta));
        }
      }
    }
  }

  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Vec2d> path_points_;           // size = n.
  std::vector<double> path_boundary_dists_;  // size = n.
  // Whether the boundary is on the left of the path.
  bool left_ = false;
  // Distances from mid control points to RAC on vehicle longitudinal axis.
  std::vector<double> mid_edges_to_center_;
  std::unique_ptr<FrenetFrame> path_;
  std::vector<double> cascade_buffers_;
  std::vector<double> cascade_gains_;
  std::vector<std::string> sub_names_;

  // Whether computes cost if point is beyond the head or tail of the path.
  bool ignore_head_ = false;
  bool ignore_tail_ = false;

  // States.
  std::array<Vec2d, PROB::kHorizon> state_normals_;
  // Distance from state points to path. Positive if the
  // point is on the left of the path.
  std::array<double, PROB::kHorizon> state_dists_;
  // Distance from path to boundary at the state
  // points. Always positive.
  std::array<double, PROB::kHorizon> state_boundary_dists_;
  std::array<std::pair<int, int>, PROB::kHorizon> state_index_pairs_;
  std::array<double, PROB::kHorizon> state_alphas_;
  std::array<bool, PROB::kHorizon> state_beyond_head_;
  std::array<bool, PROB::kHorizon> state_beyond_tail_;
  // Distance from corner points to path. Positive if the
  // point is on the left of the path.
  std::array<Vec2d, PROB::kHorizon> corner_normals_;
  // Distance from path to boundary at the corner
  // points. Always positive.
  std::array<double, PROB::kHorizon> corner_dists_;
  std::array<double, PROB::kHorizon> corner_boundary_dists_;
  std::array<std::pair<int, int>, PROB::kHorizon> corner_index_pairs_;
  std::array<double, PROB::kHorizon> corner_alphas_;
  std::array<bool, PROB::kHorizon> corner_beyond_head_;
  std::array<bool, PROB::kHorizon> corner_beyond_tail_;
  // Distance from mid edge points to path. Positive if the
  // point is on the left of the path.
  std::vector<std::array<Vec2d, PROB::kHorizon>> mid_normals_;
  // Distance from path to boundary at the mid edge
  // points. Always positive.
  std::vector<std::array<double, PROB::kHorizon>> mid_dists_;
  std::vector<std::array<double, PROB::kHorizon>> mid_boundary_dists_;
  std::vector<std::array<std::pair<int, int>, PROB::kHorizon>> mid_index_pairs_;
  std::vector<std::array<double, PROB::kHorizon>> mid_alphas_;
  std::vector<std::array<bool, PROB::kHorizon>> mid_beyond_head_;
  std::vector<std::array<bool, PROB::kHorizon>> mid_beyond_tail_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_STEP_STATIC_BOUNDARY_COST_H_
