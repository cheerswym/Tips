#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_PATH_BOUNDARY_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_PATH_BOUNDARY_COST_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

// A path boundary represented by a sequence of path points, and the
// associated boundary distances to those path points.
template <typename PROB>
class MfobPathBoundaryCost : public Cost<PROB> {
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
  MfobPathBoundaryCost(VehicleGeometryParamsProto vehicle_geometry_params,
                       const std::vector<Vec2d> &path_points,
                       const CenterLineQueryHelper<PROB> *center_line_helper,
                       std::vector<double> l_offsets,
                       std::vector<std::vector<double>> path_boundary_dists,
                       bool left, std::vector<double> mid_edges_to_center,
                       std::vector<std::vector<double>> ref_gains,
                       std::vector<std::string> sub_names, bool use_qtfm,
                       std::vector<double> buffers_min = {-1.0},
                       std::vector<double> rear_buffers_max = {1.0},
                       std::vector<double> front_buffers_max = {0.6},
                       std::vector<double> mid_buffers_max = {0.6},
                       std::vector<double> cascade_gains = {1.0},
                       std::vector<double> rear_gain = {1.0},
                       std::vector<double> front_gain = {1.0},
                       std::string name = absl::StrCat(PROB::kProblemPrefix,
                                                       "PathBoundaryCost"),
                       double scale = 1.0, bool enable_fast_math = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, enable_fast_math),
        vehicle_geometry_params_(std::move(vehicle_geometry_params)),
        path_points_(path_points),
        l_offsets_(std::move(l_offsets)),
        path_boundary_dists_(std::move(path_boundary_dists)),
        left_(left),
        mid_edges_to_center_(std::move(mid_edges_to_center)),
        ref_gains_(std::move(ref_gains)),
        cascade_gains_(std::move(cascade_gains)),
        sub_names_(std::move(sub_names)),
        center_line_helper_(center_line_helper),
        rear_gain_(rear_gain),
        front_gain_(front_gain) {
    QCHECK_GT(path_points_.size(), 1);
    QCHECK_EQ(path_boundary_dists_.size(), sub_names_.size());
    QCHECK_EQ(path_boundary_dists_.size(), cascade_gains_.size());
    QCHECK_EQ(path_boundary_dists_.size(), rear_gain_.size());
    QCHECK_EQ(path_boundary_dists_.size(), front_gain_.size());
    QCHECK_EQ(path_boundary_dists_.size(), ref_gains_.size());
    QCHECK_EQ(path_boundary_dists_.size(), buffers_min.size());
    QCHECK_EQ(path_boundary_dists_.size(), rear_buffers_max.size());
    QCHECK_EQ(path_boundary_dists_.size(), front_buffers_max.size());
    QCHECK_EQ(path_boundary_dists_.size(), mid_buffers_max.size());
    QCHECK_EQ(path_points_.size(), l_offsets_.size());
    QCHECK_EQ(path_points_.size(), path_boundary_dists_.front().size());

    if (center_line_helper_ != nullptr) {
      QCHECK_EQ(&path_points, center_line_helper_->points());
      // path_ remains nullptr, use center line helper's path instead.
    } else {
      if (use_qtfm) {
        path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
            BuildQtfmEnhancedKdTreeFrenetFrame(path_points).value());
      } else {
        path_ = std::make_unique<KdTreeFrenetFrame>(
            BuildKdTreeFrenetFrame(path_points).value());
      }
    }
    const double half_width = vehicle_geometry_params_.width() * 0.5;
    const int num_of_path_boundaries = cascade_gains_.size();

    rear_clamped_path_boundary_buffers_.resize(num_of_path_boundaries);
    front_clamped_path_boundary_buffers_.resize(num_of_path_boundaries);
    mid_clamped_path_boundary_buffers_.resize(mid_edges_to_center_.size());
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      mid_clamped_path_boundary_buffers_[i].resize(num_of_path_boundaries);
    }
    for (int i = 0; i < num_of_path_boundaries; ++i) {
      auto &rear_clamped_path_boundary_buffers =
          rear_clamped_path_boundary_buffers_[i];
      auto &front_clamped_path_boundary_buffers =
          front_clamped_path_boundary_buffers_[i];

      const int path_boundary_dists_i_size = path_boundary_dists_[i].size();
      rear_clamped_path_boundary_buffers.reserve(path_boundary_dists_i_size);
      front_clamped_path_boundary_buffers.reserve(path_boundary_dists_i_size);
      for (int j = 0; j < mid_edges_to_center_.size(); ++j) {
        mid_clamped_path_boundary_buffers_[j][i].reserve(
            path_boundary_dists_i_size);
      }
      const auto &path_boundary_dist = path_boundary_dists_[i];
      for (int k = 0; k < path_boundary_dist.size(); ++k) {
        double center_to_boundary_dist = path_boundary_dist[k];
        if (left_) {
          center_to_boundary_dist -= l_offsets_[k];
        } else {
          center_to_boundary_dist += l_offsets_[k];
        }
        rear_clamped_path_boundary_buffers.push_back(
            std::clamp(center_to_boundary_dist - half_width, buffers_min[i],
                       rear_buffers_max[i]));
        front_clamped_path_boundary_buffers.push_back(
            std::clamp(center_to_boundary_dist - half_width, buffers_min[i],
                       front_buffers_max[i]));
        for (int j = 0; j < mid_edges_to_center_.size(); ++j) {
          mid_clamped_path_boundary_buffers_[j][i].push_back(
              std::clamp(center_to_boundary_dist - half_width, buffers_min[i],
                         mid_buffers_max[i]));
        }
      }
    }
    mid_normals_.resize(mid_edges_to_center_.size());
    mid_index_pairs_.resize(mid_edges_to_center_.size());
    mid_alphas_.resize(mid_edges_to_center_.size());
    mid_dynamic_buffers_.resize(mid_edges_to_center_.size());
    mid_to_path_boundary_dists_.resize(mid_edges_to_center_.size());
    mid_gains_.resize(mid_edges_to_center_.size());

    state_dynamic_buffers_.resize(num_of_path_boundaries);
    corner_dynamic_buffers_.resize(num_of_path_boundaries);
    state_to_path_boundary_dists_.resize(num_of_path_boundaries);
    corner_to_path_boundary_dists_.resize(num_of_path_boundaries);
    state_gains_.resize(num_of_path_boundaries);
    corner_gains_.resize(num_of_path_boundaries);
    for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
      mid_dynamic_buffers_[i].resize(num_of_path_boundaries);
      mid_to_path_boundary_dists_[i].resize(num_of_path_boundaries);
      mid_gains_[i].resize(num_of_path_boundaries);
    }
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(sub_names_.size());
    for (int k = 0; k < PROB::kHorizon; ++k) {
      // Cost for rac point
      for (int j = 0; j < path_boundary_dists_.size(); ++j) {
        const double d = state_to_path_boundary_dists_[j][k];
        if (d < state_dynamic_buffers_[j][k]) {
          res.AddSubG(j, 0.5 * Cost<PROB>::scale() * state_gains_[j][k] *
                             rear_gain_[j] *
                             Sqr(state_dynamic_buffers_[j][k] - d));
        }

        // Cost for corner point
        const double d_corner = corner_to_path_boundary_dists_[j][k];
        if (d_corner < corner_dynamic_buffers_[j][k]) {
          res.AddSubG(j, 0.5 * Cost<PROB>::scale() * corner_gains_[j][k] *
                             front_gain_[j] *
                             Sqr(corner_dynamic_buffers_[j][k] - d_corner));
        }

        // Cost for mid edge point
        for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
          const double d_mid = mid_to_path_boundary_dists_[i][j][k];
          if (d_mid < mid_dynamic_buffers_[i][j][k]) {
            res.AddSubG(j, 0.5 * Cost<PROB>::scale() * mid_gains_[i][j][k] *
                               front_gain_[j] *
                               Sqr(mid_dynamic_buffers_[i][j][k] - d_mid));
          }
        }
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType &x,
                                  const ControlType &u,
                                  bool using_scale) const override {
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    for (int j = 0; j < path_boundary_dists_.size(); ++j) {
      // Cost for rac point
      const double d = state_to_path_boundary_dists_[j][k];
      if (d < state_dynamic_buffers_[j][k]) {
        res.AddSubG(j, 0.5 * Cost<PROB>::scale() * state_gains_[j][k] *
                           rear_gain_[j] *
                           Sqr(state_dynamic_buffers_[j][k] - d));
      }

      // Cost for corner point
      const double d_corner = corner_to_path_boundary_dists_[j][k];
      if (d_corner < corner_dynamic_buffers_[j][k]) {
        res.AddSubG(j, 0.5 * Cost<PROB>::scale() * corner_gains_[j][k] *
                           front_gain_[j] *
                           Sqr(corner_dynamic_buffers_[j][k] - d_corner));
      }

      // Cost for mid edge point
      for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
        const double d_mid = mid_to_path_boundary_dists_[i][j][k];
        if (d_mid < mid_dynamic_buffers_[i][j][k]) {
          res.AddSubG(j, 0.5 * Cost<PROB>::scale() * mid_gains_[i][j][k] *
                             front_gain_[j] *
                             Sqr(mid_dynamic_buffers_[i][j][k] - d_mid));
        }
      }
    }
    if (!using_scale) {
      res.VecDiv(cascade_gains_);
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    double g = 0.0;
    for (int j = 0; j < path_boundary_dists_.size(); ++j) {
      // Cost for rac point
      const double d = state_to_path_boundary_dists_[j][k];
      if (d < state_dynamic_buffers_[j][k]) {
        g += 0.5 * Cost<PROB>::scale() * state_gains_[j][k] * rear_gain_[j] *
             Sqr(state_dynamic_buffers_[j][k] - d);
      }

      // Cost for corner point
      const double d_corner = corner_to_path_boundary_dists_[j][k];
      if (d_corner < corner_dynamic_buffers_[j][k]) {
        g += 0.5 * Cost<PROB>::scale() * corner_gains_[j][k] * front_gain_[j] *
             Sqr(corner_dynamic_buffers_[j][k] - d_corner);
      }

      // Cost for mid edge point
      for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
        const double d_mid = mid_to_path_boundary_dists_[i][j][k];
        if (d_mid < mid_dynamic_buffers_[i][j][k]) {
          g += 0.5 * Cost<PROB>::scale() * mid_gains_[i][j][k] *
               front_gain_[j] * Sqr(mid_dynamic_buffers_[i][j][k] - d_mid);
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
    for (int j = 0; j < path_boundary_dists_.size(); ++j) {
      const double d = state_to_path_boundary_dists_[j][k];
      if (d < state_dynamic_buffers_[j][k]) {
        const double d_penetration = d - state_dynamic_buffers_[j][k];
        const Vec2d segment = path_points_[state_index_pairs_[k].second] -
                              path_points_[state_index_pairs_[k].first];
        const double d0 = path_boundary_dists_[j][state_index_pairs_[k].first];
        const double d1 = path_boundary_dists_[j][state_index_pairs_[k].second];
        double d_buffer = 0.0;
        if (state_alphas_[k] >= 0.0 && state_alphas_[k] <= 1.0) {
          d_buffer =
              rear_clamped_path_boundary_buffers_[j][state_index_pairs_[k]
                                                         .second] -
              rear_clamped_path_boundary_buffers_[j]
                                                 [state_index_pairs_[k].first];
        }
        const double sign = left_ ? -1.0 : 1.0;
        if (state_alphas_[k] < 0.0 || state_alphas_[k] > 1.0) {
          (*dgdx).template segment<2>(PROB::kStateXIndex) +=
              Cost<PROB>::scale() * state_gains_[j][k] * rear_gain_[j] *
              d_penetration * sign * state_normals_[k].transpose();
        } else {
          (*dgdx).template segment<2>(PROB::kStateXIndex) +=
              Cost<PROB>::scale() * state_gains_[j][k] * rear_gain_[j] *
              d_penetration *
              ((d1 - d0 - d_buffer) * segment.transpose() /
                   segment.squaredNorm() +
               sign * state_normals_[k].transpose());
        }
      }

      // Gradient for corner point
      AddEdgePointDGDx(dgdx, corner_to_path_boundary_dists_[j][k],
                       vehicle_geometry_params_.front_edge_to_center(),
                       sin_theta, cos_theta, half_width,
                       corner_dynamic_buffers_[j][k], path_boundary_dists_[j],
                       front_clamped_path_boundary_buffers_[j],
                       corner_index_pairs_[k], corner_alphas_[k],
                       corner_normals_[k], corner_gains_[j][k], front_gain_[j]);

      // Gradient for mid edge point
      for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
        AddEdgePointDGDx(
            dgdx, mid_to_path_boundary_dists_[i][j][k], mid_edges_to_center_[i],
            sin_theta, cos_theta, half_width, mid_dynamic_buffers_[i][j][k],
            path_boundary_dists_[j], mid_clamped_path_boundary_buffers_[i][j],
            mid_index_pairs_[i][k], mid_alphas_[i][k], mid_normals_[i][k],
            mid_gains_[i][j][k], front_gain_[j]);
      }
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
    for (int j = 0; j < path_boundary_dists_.size(); ++j) {
      const double d = state_to_path_boundary_dists_[j][k];
      if (d < state_dynamic_buffers_[j][k]) {
        const Vec2d segment = path_points_[state_index_pairs_[k].second] -
                              path_points_[state_index_pairs_[k].first];
        const double d0 = path_boundary_dists_[j][state_index_pairs_[k].first];
        const double d1 = path_boundary_dists_[j][state_index_pairs_[k].second];
        double d_buffer = 0.0;
        if (state_alphas_[k] >= 0.0 && state_alphas_[k] <= 1.0) {
          d_buffer =
              rear_clamped_path_boundary_buffers_[j][state_index_pairs_[k]
                                                         .second] -
              rear_clamped_path_boundary_buffers_[j]
                                                 [state_index_pairs_[k].first];
        }
        const double sign = left_ ? -1.0 : 1.0;
        if (state_alphas_[k] < 0.0 || state_alphas_[k] > 1.0) {
          (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                          PROB::kStateXIndex) +=
              Cost<PROB>::scale() * state_gains_[j][k] * rear_gain_[j] *
              state_normals_[k] * state_normals_[k].transpose();
        } else {
          const Vec2d vec =
              (d1 - d0 - d_buffer) * segment / segment.squaredNorm() +
              sign * state_normals_[k];
          (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                          PROB::kStateXIndex) +=
              Cost<PROB>::scale() * state_gains_[j][k] * rear_gain_[j] * vec *
              vec.transpose();
        }
      }

      // Hessian for corner point.
      AddEdgePointDDGDxDx(
          ddgdxdx, corner_to_path_boundary_dists_[j][k],
          vehicle_geometry_params_.front_edge_to_center(), sin_theta, cos_theta,
          half_width, corner_dynamic_buffers_[j][k], path_boundary_dists_[j],
          front_clamped_path_boundary_buffers_[j], corner_index_pairs_[k],
          corner_alphas_[k], corner_normals_[k], corner_gains_[j][k],
          front_gain_[j]);

      // Hessian for mid point.
      for (int i = 0; i < mid_edges_to_center_.size(); ++i) {
        AddEdgePointDDGDxDx(
            ddgdxdx, mid_to_path_boundary_dists_[i][j][k],
            mid_edges_to_center_[i], sin_theta, cos_theta, half_width,
            mid_dynamic_buffers_[i][j][k], path_boundary_dists_[j],
            mid_clamped_path_boundary_buffers_[i][j], mid_index_pairs_[i][k],
            mid_alphas_[i][k], mid_normals_[i][k], mid_gains_[i][j][k],
            front_gain_[j]);
      }
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

    const double half_width = vehicle_geometry_params_.width() * 0.5;
    const auto get_rac_to_path_boundary_distance =
        [&half_width, this](double boundary_dist, double state_dist) -> double {
      return this->left_ ? boundary_dist - state_dist - half_width
                         : boundary_dist + state_dist - half_width;
    };
    // Update normal and dist of rac at each state step
    if (center_line_helper_ == nullptr) {
      Update(GetStatePosAtStep, get_rac_to_path_boundary_distance,
             rear_clamped_path_boundary_buffers_, xs, &state_normals_,
             &state_dynamic_buffers_, &state_to_path_boundary_dists_,
             &state_gains_, &state_index_pairs_, &state_alphas_);
    } else {
      UpdateWithCenterLineHelper(get_rac_to_path_boundary_distance,
                                 rear_clamped_path_boundary_buffers_, xs,
                                 &state_normals_, &state_dynamic_buffers_,
                                 &state_to_path_boundary_dists_, &state_gains_,
                                 &state_index_pairs_, &state_alphas_);
    }
    if (left_) {
      VLOG(4) << "Update corner for left boundary.";
    } else {
      VLOG(4) << "Update corner for right boundary.";
    }

    const auto get_edge_point_to_path_boundary_distance =
        [this](double boundary_dist, double state_dist) -> double {
      return this->left_ ? boundary_dist - state_dist
                         : boundary_dist + state_dist;
    };
    // Update normal and dist of corner at each state step
    Update(std::bind(GetEdgePointPosAtStep, std::placeholders::_1,
                     std::placeholders::_2,
                     vehicle_geometry_params_.front_edge_to_center(),
                     vehicle_geometry_params_, left_,
                     Cost<PROB>::enable_fast_math()),
           get_edge_point_to_path_boundary_distance,
           front_clamped_path_boundary_buffers_, xs, &corner_normals_,
           &corner_dynamic_buffers_, &corner_to_path_boundary_dists_,
           &corner_gains_, &corner_index_pairs_, &corner_alphas_);
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
             get_edge_point_to_path_boundary_distance,
             mid_clamped_path_boundary_buffers_[i], xs, &mid_normals_[i],
             &mid_dynamic_buffers_[i], &mid_to_path_boundary_dists_[i],
             &mid_gains_[i], &mid_index_pairs_[i], &mid_alphas_[i]);
    }
  }

 private:
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

  void UpdateWithCenterLineHelper(
      const std::function<double(double, double)>
          &get_distance_to_path_boundary,
      const std::vector<std::vector<double>> &clamped_path_boundary_buffers,
      const StatesType &xs, std::array<Vec2d, PROB::kHorizon> *normals,
      std::vector<std::array<double, PROB::kHorizon>> *dynamic_buffer,
      std::vector<std::array<double, PROB::kHorizon>> *to_path_boundary_dists,
      std::vector<std::array<double, PROB::kHorizon>> *gains,
      std::array<std::pair<int, int>, PROB::kHorizon> *index_pairs,
      std::array<double, PROB::kHorizon> *alphas) {
    const int buffer_count = to_path_boundary_dists->size();
    const auto &rac_alphas = center_line_helper_->alphas();
    const auto &rac_s_l_list = center_line_helper_->s_l_list();
    const auto &rac_normals = center_line_helper_->normals();
    const auto &rac_index_pairs = center_line_helper_->index_pairs();

    for (int k = 0; k < PROB::kHorizon; ++k) {
      const FrenetCoordinate sl = rac_s_l_list[k];
      const Vec2d &normal = rac_normals[k];
      const std::pair<int, int> &index_pair = rac_index_pairs[k];
      const double alpha = rac_alphas[k];
      (*normals)[k] = normal;
      (*index_pairs)[k] = index_pair;
      (*alphas)[k] = alpha;
      for (int i = 0; i < buffer_count; ++i) {
        if (alpha < 0.0) {
          (*dynamic_buffer)[i][k] =
              clamped_path_boundary_buffers[i][index_pair.first];
          (*to_path_boundary_dists)[i][k] = get_distance_to_path_boundary(
              path_boundary_dists_[i][index_pair.first], sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.first] * cascade_gains_[i];

        } else if (alpha > 1.0) {
          (*dynamic_buffer)[i][k] =
              clamped_path_boundary_buffers[i][index_pair.second];
          (*to_path_boundary_dists)[i][k] = get_distance_to_path_boundary(
              path_boundary_dists_[i][index_pair.second], sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.second] * cascade_gains_[i];
        } else {
          (*dynamic_buffer)[i][k] =
              Lerp(clamped_path_boundary_buffers[i][index_pair.first],
                   clamped_path_boundary_buffers[i][index_pair.second], alpha);
          const double boundary_dist =
              Lerp(path_boundary_dists_[i][index_pair.first],
                   path_boundary_dists_[i][index_pair.second], alpha);
          (*to_path_boundary_dists)[i][k] =
              get_distance_to_path_boundary(boundary_dist, sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.first] * cascade_gains_[i];
        }
      }
      if (UNLIKELY(VLOG_IS_ON(4))) {
        VLOG(4) << "Dists [" << k << "]: " << sl.l;
        VLOG(4) << "Normal [" << k << "]: " << normals->at(k).x() << " "
                << normals->at(k).y();
        VLOG(4) << "Index pair [" << k << "]: " << index_pairs->at(k).first
                << " " << index_pairs->at(k).second;
        VLOG(4) << "Alpha [" << k << "]: " << alphas->at(k);
        for (int i = 0; i < buffer_count; ++i) {
          VLOG(4) << "To boundary dists [" << i << "][" << k
                  << "]: " << (*to_path_boundary_dists)[i][k];
          VLOG(4) << "Dynamic buffer [" << i << "][" << k
                  << "]: " << (*dynamic_buffer)[i][k];
        }
      }
    }
  }

  const FrenetFrame &GetPath() const {
    if (center_line_helper_ != nullptr) {
      return center_line_helper_->path();
    } else {
      QCHECK_NOTNULL(path_);
      return *path_;
    }
  }

  void Update(
      const std::function<Vec2d(const StatesType &, int)> &get_pos_at_step,
      const std::function<double(double, double)>
          &get_distance_to_path_boundary,
      const std::vector<std::vector<double>> &clamped_path_boundary_buffers,
      const StatesType &xs, std::array<Vec2d, PROB::kHorizon> *normals,
      std::vector<std::array<double, PROB::kHorizon>> *dynamic_buffer,
      std::vector<std::array<double, PROB::kHorizon>> *to_path_boundary_dists,
      std::vector<std::array<double, PROB::kHorizon>> *gains,
      std::array<std::pair<int, int>, PROB::kHorizon> *index_pairs,
      std::array<double, PROB::kHorizon> *alphas) const {
    const int buffer_count = to_path_boundary_dists->size();
    for (int k = 0; k < PROB::kHorizon; ++k) {
      const Vec2d xy = get_pos_at_step(xs, k);
      FrenetCoordinate sl;
      Vec2d normal;
      std::pair<int, int> index_pair;
      double alpha = 0.0;

      GetPath().XYToSL(xy, &sl, &normal, &index_pair, &alpha);

      (*normals)[k] = normal;
      (*index_pairs)[k] = index_pair;
      (*alphas)[k] = alpha;
      for (int i = 0; i < buffer_count; ++i) {
        if (alpha < 0.0) {
          (*dynamic_buffer)[i][k] =
              clamped_path_boundary_buffers[i][index_pair.first];
          (*to_path_boundary_dists)[i][k] = get_distance_to_path_boundary(
              path_boundary_dists_[i][index_pair.first], sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.first] * cascade_gains_[i];

        } else if (alpha > 1.0) {
          (*dynamic_buffer)[i][k] =
              clamped_path_boundary_buffers[i][index_pair.second];
          (*to_path_boundary_dists)[i][k] = get_distance_to_path_boundary(
              path_boundary_dists_[i][index_pair.second], sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.second] * cascade_gains_[i];
        } else {
          (*dynamic_buffer)[i][k] =
              Lerp(clamped_path_boundary_buffers[i][index_pair.first],
                   clamped_path_boundary_buffers[i][index_pair.second], alpha);
          const double boundary_dist =
              Lerp(path_boundary_dists_[i][index_pair.first],
                   path_boundary_dists_[i][index_pair.second], alpha);
          (*to_path_boundary_dists)[i][k] =
              get_distance_to_path_boundary(boundary_dist, sl.l);
          (*gains)[i][k] = ref_gains_[i][index_pair.first] * cascade_gains_[i];
        }
      }
      if (UNLIKELY(VLOG_IS_ON(4))) {
        VLOG(4) << "Dists [" << k << "]: " << sl.l;
        VLOG(4) << "Normal [" << k << "]: " << normals->at(k).x() << " "
                << normals->at(k).y();
        VLOG(4) << "Index pair [" << k << "]: " << index_pairs->at(k).first
                << " " << index_pairs->at(k).second;
        VLOG(4) << "Alpha [" << k << "]: " << alphas->at(k);
        for (int i = 0; i < buffer_count; ++i) {
          VLOG(4) << "To boundary dists [" << i << "][" << k
                  << "]: " << (*to_path_boundary_dists)[i][k];
          VLOG(4) << "Dynamic buffer [" << i << "][" << k
                  << "]: " << (*dynamic_buffer)[i][k];
        }
      }
    }
  }

  void AddEdgePointDGDx(
      DGDxType *dgdx, double d_corner, double dist_to_rac, double sin_theta,
      double cos_theta, double half_width, double corner_dynamic_buffer,
      const std::vector<double> &path_boundary_dists,
      const std::vector<double> &clamped_path_boundary_buffers,
      const std::pair<int, int> &corner_index_pair, double corner_alpha,
      const Vec2d &corner_normal, double gain, double point_gain) const {
    Vec2d d_delta;
    if (!left_) {
      d_delta[0] = -dist_to_rac * sin_theta + half_width * cos_theta;
      d_delta[1] = dist_to_rac * cos_theta + half_width * sin_theta;
    } else {
      d_delta[0] = -dist_to_rac * sin_theta - half_width * cos_theta;
      d_delta[1] = dist_to_rac * cos_theta - half_width * sin_theta;
    }
    if (d_corner < corner_dynamic_buffer) {
      const double d_penetration = d_corner - corner_dynamic_buffer;
      const Vec2d segment = path_points_[corner_index_pair.second] -
                            path_points_[corner_index_pair.first];
      const double d0 = path_boundary_dists[corner_index_pair.first];
      const double d1 = path_boundary_dists[corner_index_pair.second];
      double d_buffer = 0.0;
      if (corner_alpha >= 0.0 && corner_alpha <= 1.0) {
        d_buffer = clamped_path_boundary_buffers[corner_index_pair.second] -
                   clamped_path_boundary_buffers[corner_index_pair.first];
      }
      const double sign = left_ ? -1.0 : 1.0;
      if (corner_alpha < 0.0 || corner_alpha > 1.0) {
        (*dgdx).template segment<2>(0) += Cost<PROB>::scale() * gain *
                                          point_gain * d_penetration * sign *
                                          corner_normal.transpose();
        (*dgdx)[2] += Cost<PROB>::scale() * gain * point_gain * d_penetration *
                      sign * corner_normal.dot(d_delta);
      } else {
        const Vec2d vec =
            (d1 - d0 - d_buffer) * segment / segment.squaredNorm() +
            sign * corner_normal;
        (*dgdx).template segment<2>(0) +=
            Cost<PROB>::scale() * gain * point_gain * d_penetration *
            ((d1 - d0 - d_buffer) * segment.transpose() /
                 segment.squaredNorm() +
             sign * corner_normal.transpose());
        (*dgdx)[2] += Cost<PROB>::scale() * gain * point_gain * d_penetration *
                      vec.dot(d_delta);
      }
    }
  }

  void AddEdgePointDDGDxDx(
      DDGDxDxType *ddgdxdx, double d_corner, double dist_to_rac,
      double sin_theta, double cos_theta, double half_width,
      double corner_dynamic_buffer,
      const std::vector<double> &path_boundary_dists,
      const std::vector<double> &clamped_path_boundary_buffers,
      const std::pair<int, int> &corner_index_pair, double corner_alpha,
      const Vec2d &corner_normal, double gain, double point_gain) const {
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
    if (d_corner < corner_dynamic_buffer) {
      const double d_penetration = d_corner - corner_dynamic_buffer;
      const Vec2d segment = path_points_[corner_index_pair.second] -
                            path_points_[corner_index_pair.first];
      const double d0 = path_boundary_dists[corner_index_pair.first];
      const double d1 = path_boundary_dists[corner_index_pair.second];
      double d_buffer = 0.0;
      if (corner_alpha >= 0.0 && corner_alpha <= 1.0) {
        d_buffer = clamped_path_boundary_buffers[corner_index_pair.second] -
                   clamped_path_boundary_buffers[corner_index_pair.first];
      }
      const double sign = left_ ? -1.0 : 1.0;
      if (corner_alpha < 0.0 || corner_alpha > 1.0) {
        (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                        PROB::kStateXIndex) +=
            Cost<PROB>::scale() * gain * point_gain * corner_normal *
            corner_normal.transpose();
        (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                        PROB::kStateThetaIndex) +=
            Cost<PROB>::scale() * gain * point_gain * corner_normal *
            corner_normal.dot(d_delta);
        (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                        PROB::kStateXIndex) +=
            Cost<PROB>::scale() * gain * point_gain *
            corner_normal.transpose() * corner_normal.dot(d_delta);
        (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
            Cost<PROB>::scale() * gain * point_gain *
            (Sqr(corner_normal.dot(d_delta)) +
             d_penetration * sign * corner_normal.dot(dd_delta));
      } else {
        const Vec2d vec =
            (d1 - d0 - d_buffer) * segment / segment.squaredNorm() +
            sign * corner_normal;
        (*ddgdxdx).template block<2, 2>(PROB::kStateXIndex,
                                        PROB::kStateXIndex) +=
            Cost<PROB>::scale() * gain * point_gain * vec * vec.transpose();
        (*ddgdxdx).template block<2, 1>(PROB::kStateXIndex,
                                        PROB::kStateThetaIndex) +=
            Cost<PROB>::scale() * gain * point_gain * vec * vec.dot(d_delta);
        (*ddgdxdx).template block<1, 2>(PROB::kStateThetaIndex,
                                        PROB::kStateXIndex) +=
            Cost<PROB>::scale() * gain * point_gain * vec.transpose() *
            vec.dot(d_delta);
        (*ddgdxdx)(PROB::kStateThetaIndex, PROB::kStateThetaIndex) +=
            Cost<PROB>::scale() * gain * point_gain *
            (Sqr(vec.dot(d_delta)) + d_penetration * vec.dot(dd_delta));
      }
    }
  }

  VehicleGeometryParamsProto vehicle_geometry_params_;
  std::vector<Vec2d> path_points_;  // size = n.
  // Inner vectors are distances, size = n, outer vector is multi layer path
  // boundaries.
  std::vector<double> l_offsets_;
  std::vector<std::vector<double>> path_boundary_dists_;
  std::vector<std::vector<double>> rear_clamped_path_boundary_buffers_;
  std::vector<std::vector<double>> front_clamped_path_boundary_buffers_;
  std::vector<std::vector<std::vector<double>>>
      mid_clamped_path_boundary_buffers_;
  // Whether the boundary is on the left of the path.
  bool left_ = false;
  // Distances from mid control points to RAC on vehicle longitudinal axis.
  std::vector<double> mid_edges_to_center_;

  // Gains for each station of every path boudnary.
  std::vector<std::vector<double>> ref_gains_;

  std::vector<double> cascade_gains_;
  std::vector<std::string> sub_names_;

  std::unique_ptr<FrenetFrame> path_;
  const CenterLineQueryHelper<PROB> *center_line_helper_;

  std::vector<double> rear_gain_;
  std::vector<double> front_gain_;

  // States.
  std::array<Vec2d, PROB::kHorizon> state_normals_;
  std::array<std::pair<int, int>, PROB::kHorizon> state_index_pairs_;
  std::array<double, PROB::kHorizon> state_alphas_;
  std::vector<std::array<double, PROB::kHorizon>> state_dynamic_buffers_;
  // Distance from rac to path boundary.
  std::vector<std::array<double, PROB::kHorizon>> state_to_path_boundary_dists_;
  std::vector<std::array<double, PROB::kHorizon>> state_gains_;

  std::array<Vec2d, PROB::kHorizon> corner_normals_;
  std::array<std::pair<int, int>, PROB::kHorizon> corner_index_pairs_;
  std::array<double, PROB::kHorizon> corner_alphas_;
  std::vector<std::array<double, PROB::kHorizon>> corner_dynamic_buffers_;
  // Distance from corner points to path boundary.
  std::vector<std::array<double, PROB::kHorizon>>
      corner_to_path_boundary_dists_;
  std::vector<std::array<double, PROB::kHorizon>> corner_gains_;

  std::vector<std::array<Vec2d, PROB::kHorizon>> mid_normals_;
  std::vector<std::array<std::pair<int, int>, PROB::kHorizon>> mid_index_pairs_;
  std::vector<std::array<double, PROB::kHorizon>> mid_alphas_;
  std::vector<std::vector<std::array<double, PROB::kHorizon>>>
      mid_dynamic_buffers_;
  // Distance from corner points to path boundary.
  std::vector<std::vector<std::array<double, PROB::kHorizon>>>
      mid_to_path_boundary_dists_;
  std::vector<std::vector<std::array<double, PROB::kHorizon>>> mid_gains_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_MFOB_PATH_BOUNDARY_COST_H_
