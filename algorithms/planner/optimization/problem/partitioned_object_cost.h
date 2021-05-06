#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_OBSTACLE_COST_H_

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/types/span.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/optimization/problem/av_model_helper.h"
#include "onboard/planner/optimization/problem/cost.h"

namespace qcraft {
namespace planner {

template <typename PROB>
class PartitionedObjectCost : public Cost<PROB> {
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

  struct Object {
    std::vector<Segment2d> lines;
    std::vector<double> buffers;
    std::vector<double> gains;
    Vec2d ref_x;
    double offset;
    Vec2d ref_tangent;
    // If object is enabled
    bool enable = false;
  };

  static constexpr double kNormalizedScale = 1.0;
  PartitionedObjectCost(std::vector<Object> objects, double dist_to_rac,
                        double angle_to_axis,
                        std::vector<double> cascade_buffers,
                        const AvModelHelper<PROB> *av_model_helper,
                        std::vector<std::string> sub_names,
                        std::string name = absl::StrCat(
                            PROB::kProblemPrefix, "PartitionedObjectCost"),
                        double scale = 1.0, bool enable_fast_math = false)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, enable_fast_math),
        objects_(std::move(objects)),
        num_objects_(objects_.size()),
        dist_to_rac_(dist_to_rac),
        angle_to_axis_(angle_to_axis),
        cascade_buffers_(cascade_buffers),
        av_model_helper_(av_model_helper),
        sub_names_(std::move(sub_names)) {
    QCHECK_EQ(cascade_buffers_.size(), sub_names_.size());
    penetrations_.resize(num_objects_);
    penetration_jacobians_.resize(num_objects_,
                                  PenetrationJacobianType::Zero());
    penetration_hessians_.resize(num_objects_, PenetrationHessianType::Zero());
    for (int i = 0, n = num_objects_; i < n; ++i) {
      // Object prediction time may be shorter than ddp time horizon.
      auto &ob = objects_[i];
      QCHECK_EQ(ob.buffers.size(), ob.gains.size());
      QCHECK_GE(cascade_buffers_.size(), ob.gains.size());
      auto &penetrations_i = penetrations_[i];
      const auto ob_buffers = absl::MakeConstSpan(ob.buffers);
      penetrations_i.reserve(ob_buffers.size());
      for (int k = 0; k < ob_buffers.size(); ++k) {
        penetrations_i.push_back(std::numeric_limits<double>::infinity());
      }
    }
    if (av_model_helper_ != nullptr) {
      QCHECK_NEAR(dist_to_rac_, av_model_helper_->dist_to_rac(), 1e-9);
      QCHECK_NEAR(angle_to_axis_, av_model_helper_->angle_to_axis(), 1e-9);
    }
  }

  DividedG SumGForAllSteps(const StatesType &xs,
                           const ControlsType &us) const override {
    DividedG res(sub_names_.size());
    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (k >= num_objects_) break;
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
      const auto &ob_k = objects_[k];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(
              i, cascade_buffers_[i] * ob_k.gains[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType &x,
                                  const ControlType &u,
                                  bool using_scale) const override {
    DCHECK_GE(k, 0);
    std::vector<double> cascade_buffers(cascade_buffers_.size());
    if (using_scale) {
      cascade_buffers = cascade_buffers_;
    } else {
      std::fill(cascade_buffers.begin(), cascade_buffers.end(), 1.0);
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    if (k >= num_objects_) return res;
    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto &ob_k = objects_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        res.AddSubG(i, 0.5 * Cost<PROB>::scale() * cascade_buffers[i] *
                           ob_k.gains[i] * Sqr(penetrations_k[i]));
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType &x,
                   const ControlType &u) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return 0.0;

    double g = 0.0;
    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto &ob_k = objects_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        g += 0.5 * Cost<PROB>::scale() * cascade_buffers_[i] * ob_k.gains[i] *
             Sqr(penetrations_k[i]);
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType &x, const ControlType &u,
               DGDxType *dgdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;

    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto &ob_k = objects_[k];
    const auto &penetration_jacobians_k = penetration_jacobians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *dgdx += Cost<PROB>::scale() * cascade_buffers_[i] * ob_k.gains[i] *
                 penetrations_k[i] * penetration_jacobians_k;
      }
    }
  }
  void AddDGDu(int k, const StateType &x, const ControlType &u,
               DGDuType *dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType &x, const ControlType &u,
                  DDGDxDxType *ddgdxdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;

    const auto penetrations_k = absl::MakeConstSpan(penetrations_[k]);
    const auto &ob_k = objects_[k];
    const auto &penetration_jacobians_k = penetration_jacobians_[k];
    const auto &penetration_hessians_k = penetration_hessians_[k];
    for (int i = 0; i < penetrations_k.size(); ++i) {
      if (penetrations_k[i] < 0.0) {
        *ddgdxdx +=
            Cost<PROB>::scale() * cascade_buffers_[i] * ob_k.gains[i] *
            (penetrations_k[i] * penetration_hessians_k +
             penetration_jacobians_k.transpose() * penetration_jacobians_k);
      }
    }
  }
  void AddDDGDuDx(int k, const StateType &x, const ControlType &u,
                  DDGDuDxType *ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType &x, const ControlType &u,
                  DDGDuDuType *ddgdudu) const override {}

  void Update(const StatesType &xs, const ControlsType &us,
              bool value_only) override {
    if (av_model_helper_ == nullptr) {
      UpdateWithoutAvModelHelper(xs, us, value_only);
    } else {
      UpdateWithAvModelHelper(xs, us, value_only);
    }
  }

  void UpdateWithoutAvModelHelper(const StatesType &xs, const ControlsType &us,
                                  bool value_only) {
    const auto compute_point_cross_prod_on_lines =
        [](absl::Span<const Segment2d> lines,
           const Vec2d &pos) -> std::vector<double> {
      std::vector<double> cross_prods;
      cross_prods.reserve(lines.size());
      for (const auto &line : lines) {
        cross_prods.emplace_back(line.ProductOntoUnit(pos));
      }
      return cross_prods;
    };

    const auto compute_pos_partition =
        [](absl::Span<const Segment2d> lines, const Vec2d &pos,
           absl::Span<const double> cross_pod, int *index) -> bool {
      bool is_in_corner_region = false;
      const double prod_start = lines[0].ProjectOntoUnit(pos);
      const double prod_end = lines.back().ProjectOntoUnit(pos);
      if (prod_start < 0.0 && cross_pod[0] < 0.0) {
        *index = 0;
      } else {
        if (prod_end > lines.back().length() && cross_pod.back() < 0.0) {
          *index = lines.size() - 1;
        } else {
          for (int i = 0; i < lines.size(); ++i) {
            const Segment2d &line = lines[i];
            const double dot = line.ProjectOntoUnit(pos);
            if (dot >= 0.0 && dot <= line.length() && cross_pod[i] <= 0) {
              *index = i;
              break;
            }
            if (dot < 0.0 && i != 0) {
              *index = i;
              is_in_corner_region = true;
              break;
            }
          }
        }
      }
      return is_in_corner_region;
    };

    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (k >= num_objects_) break;
      auto penetrations_k = absl::MakeSpan(penetrations_[k]);
      auto &penetration_jacobians_k = penetration_jacobians_[k];
      auto &penetration_hessians_k = penetration_hessians_[k];

      for (int i = 0; i < penetrations_k.size(); ++i) {
        penetrations_k[i] = std::numeric_limits<double>::infinity();
      }
      if (!value_only) {
        penetration_jacobians_[k].template segment<3>(PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 1>::Zero();
        penetration_hessians_[k].template block<3, 3>(PROB::kStateXIndex,
                                                      PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 3>::Zero();
      }
      const Object &ob = objects_[k];
      if (!ob.enable) continue;

      const auto x0 = PROB::GetStateAtStep(xs, k);
      const Vec2d pos = PROB::pos(xs, k);
      const Vec2d tangent =
          Cost<PROB>::enable_fast_math()
              ? Vec2d::FastUnitFromAngle(PROB::theta(xs, k) + angle_to_axis_)
              : Vec2d::UnitFromAngle(PROB::theta(xs, k) + +angle_to_axis_);
      const Vec2d normal = tangent.Perp();
      VLOG(3) << "Step " << k;
      VLOG(4) << "x0 = " << x0.transpose()
              << " tangent = " << tangent.transpose();

      const Vec2d x_fac = pos + tangent * dist_to_rac_;
      // Filter
      if (std::abs((x_fac - ob.ref_x).dot(ob.ref_tangent)) > ob.offset) {
        continue;
      }

      const auto lines = absl::MakeConstSpan(ob.lines);
      const auto buffers = absl::MakeConstSpan(ob.buffers);

      // Cross product to inner or outer.
      std::vector<double> cross_prods =
          compute_point_cross_prod_on_lines(lines, x_fac);
      const bool in_object = absl::c_all_of(
          cross_prods, [](double cross_prod) { return cross_prod >= 0.0; });
      if (in_object) {
        // Find min_dist.
        auto min_iter = absl::c_min_element(cross_prods);
        const int min_index = std::distance(cross_prods.begin(), min_iter);
        const Segment2d &line = lines[min_index];
        const Vec2d &obs_unit_dir_perp = line.unit_direction().Perp();
        const double dist = -(x_fac - line.start()).dot(obs_unit_dir_perp);
        for (int i = 0; i < penetrations_k.size(); ++i) {
          penetrations_k[i] = dist - buffers[i];
        }
        if (!value_only) {
          penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
              -obs_unit_dir_perp;
          if (dist_to_rac_ != 0.0) {
            penetration_jacobians_k(PROB::kStateThetaIndex) =
                -obs_unit_dir_perp.dot(normal * dist_to_rac_);
            penetration_hessians_k(PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
                -obs_unit_dir_perp.dot(-tangent * dist_to_rac_);
          }
        }
      } else {
        // Find location.
        int index = 0;
        const bool is_in_corner_region =
            compute_pos_partition(lines, x_fac, cross_prods, &index);

        const Segment2d &line = lines[index];
        // prod >= 0.0 && prod <= length
        if (!is_in_corner_region) {
          const Vec2d obs_unit_dir_perp = line.unit_direction().Perp();
          const double dist = -(x_fac - line.start()).dot(obs_unit_dir_perp);
          for (int i = 0; i < penetrations_k.size(); ++i) {
            penetrations_k[i] = dist - buffers[i];
          }
          if (!value_only) {
            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                -obs_unit_dir_perp;
            if (dist_to_rac_ != 0.0) {
              penetration_jacobians_k(PROB::kStateThetaIndex) =
                  -obs_unit_dir_perp.dot(normal * dist_to_rac_);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateThetaIndex) =
                  -obs_unit_dir_perp.dot(-tangent * dist_to_rac_);
            }
          }
        } else {
          // prod <=0.0
          const Vec2d diff_x1_fac = x_fac - line.start();
          const double diff_x1_fac_norm = diff_x1_fac.norm();
          const double diff_x1_fac_norm_div = 1.0 / diff_x1_fac_norm;
          const double sqrtqrdiff_x1 = Cube(diff_x1_fac_norm_div);
          for (int i = 0; i < penetrations_k.size(); ++i) {
            penetrations_k[i] = diff_x1_fac_norm - buffers[i];
          }
          if (!value_only) {
            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                diff_x1_fac * diff_x1_fac_norm_div;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex) =
                diff_x1_fac_norm_div -
                diff_x1_fac.x() * diff_x1_fac.x() * sqrtqrdiff_x1;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex) =
                -diff_x1_fac.x() * diff_x1_fac.y() * sqrtqrdiff_x1;
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex) =
                diff_x1_fac_norm_div -
                diff_x1_fac.y() * diff_x1_fac.y() * sqrtqrdiff_x1;
            if (dist_to_rac_ != 0.0) {
              penetration_jacobians_k(PROB::kStateThetaIndex) =
                  diff_x1_fac.dot(normal * dist_to_rac_) * diff_x1_fac_norm_div;
              penetration_hessians_k(PROB::kStateXIndex,
                                     PROB::kStateThetaIndex) =
                  normal.x() * dist_to_rac_ * diff_x1_fac_norm_div -
                  sqrtqrdiff_x1 * diff_x1_fac.x() *
                      diff_x1_fac.dot(normal * dist_to_rac_);
              penetration_hessians_k(PROB::kStateYIndex,
                                     PROB::kStateThetaIndex) =
                  normal.y() * dist_to_rac_ * diff_x1_fac_norm_div -
                  sqrtqrdiff_x1 * diff_x1_fac.y() *
                      diff_x1_fac.dot(normal * dist_to_rac_);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateXIndex) =
                  penetration_hessians_k(PROB::kStateXIndex,
                                         PROB::kStateThetaIndex);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateYIndex) =
                  penetration_hessians_k(PROB::kStateYIndex,
                                         PROB::kStateThetaIndex);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateThetaIndex) =
                  (diff_x1_fac.dot(-tangent * dist_to_rac_) +
                   dist_to_rac_ * normal.dot(normal * dist_to_rac_)) *
                      diff_x1_fac_norm_div -
                  sqrtqrdiff_x1 * diff_x1_fac.dot(normal * dist_to_rac_) *
                      diff_x1_fac.dot(normal * dist_to_rac_);
            }
          }
        }
      }
    }
  }

  void UpdateWithAvModelHelper(const StatesType &xs, const ControlsType &us,
                               bool value_only) {
    const auto compute_point_cross_prod_on_lines =
        [](absl::Span<const Segment2d> lines,
           const Vec2d &pos) -> std::vector<double> {
      std::vector<double> cross_prods;
      cross_prods.reserve(lines.size());
      for (const auto &line : lines) {
        cross_prods.emplace_back(line.ProductOntoUnit(pos));
      }
      return cross_prods;
    };

    const auto compute_pos_partition =
        [](absl::Span<const Segment2d> lines, const Vec2d &pos,
           absl::Span<const double> cross_pod, int *index) -> bool {
      bool is_in_corner_region = false;
      const double prod_start = lines[0].ProjectOntoUnit(pos);
      const double prod_end = lines.back().ProjectOntoUnit(pos);
      if (prod_start < 0.0 && cross_pod[0] < 0.0) {
        *index = 0;
      } else {
        if (prod_end > lines.back().length() && cross_pod.back() < 0.0) {
          *index = lines.size() - 1;
        } else {
          for (int i = 0; i < lines.size(); ++i) {
            const Segment2d &line = lines[i];
            const double dot = line.ProjectOntoUnit(pos);
            if (dot >= 0.0 && dot <= line.length() && cross_pod[i] <= 0) {
              *index = i;
              break;
            }
            if (dot < 0.0 && i != 0) {
              *index = i;
              is_in_corner_region = true;
              break;
            }
          }
        }
      }
      return is_in_corner_region;
    };

    const std::vector<Vec2d> &centers = av_model_helper_->centers();
    const std::vector<Vec2d> &tangents = av_model_helper_->tangents();
    const std::vector<Vec2d> &normals = av_model_helper_->normals();

    for (int k = 0; k < PROB::kHorizon; ++k) {
      if (k >= num_objects_) break;
      auto penetrations_k = absl::MakeSpan(penetrations_[k]);
      auto &penetration_jacobians_k = penetration_jacobians_[k];
      auto &penetration_hessians_k = penetration_hessians_[k];

      for (int i = 0; i < penetrations_k.size(); ++i) {
        penetrations_k[i] = std::numeric_limits<double>::infinity();
      }
      if (!value_only) {
        penetration_jacobians_[k].template segment<3>(PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 1>::Zero();
        penetration_hessians_[k].template block<3, 3>(PROB::kStateXIndex,
                                                      PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 3>::Zero();
      }
      const Object &ob = objects_[k];
      if (!ob.enable) continue;

      const Vec2d &tangent = tangents[k];
      const Vec2d &normal = normals[k];
      const Vec2d &x_fac = centers[k];

      // Filter
      if (std::abs((x_fac - ob.ref_x).dot(ob.ref_tangent)) > ob.offset) {
        continue;
      }

      const auto lines = absl::MakeConstSpan(ob.lines);
      const auto buffers = absl::MakeConstSpan(ob.buffers);

      // Cross product to inner or outer.
      std::vector<double> cross_prods =
          compute_point_cross_prod_on_lines(lines, x_fac);
      const bool in_object = absl::c_all_of(
          cross_prods, [](double cross_prod) { return cross_prod >= 0.0; });
      if (in_object) {
        // Find min_dist.
        auto min_iter = absl::c_min_element(cross_prods);
        const int min_index = std::distance(cross_prods.begin(), min_iter);
        const Segment2d &line = lines[min_index];
        const Vec2d &obs_unit_dir_perp = line.unit_direction().Perp();
        const double dist = -(x_fac - line.start()).dot(obs_unit_dir_perp);
        for (int i = 0; i < penetrations_k.size(); ++i) {
          penetrations_k[i] = dist - buffers[i];
        }
        if (!value_only) {
          penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
              -obs_unit_dir_perp;
          if (dist_to_rac_ != 0.0) {
            penetration_jacobians_k(PROB::kStateThetaIndex) =
                -obs_unit_dir_perp.dot(normal * dist_to_rac_);
            penetration_hessians_k(PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
                -obs_unit_dir_perp.dot(-tangent * dist_to_rac_);
          }
        }
      } else {
        // Find location.
        int index = 0;
        const bool is_in_corner_region =
            compute_pos_partition(lines, x_fac, cross_prods, &index);

        const Segment2d &line = lines[index];
        // prod >= 0.0 && prod <= length
        if (!is_in_corner_region) {
          const Vec2d obs_unit_dir_perp = line.unit_direction().Perp();
          const double dist = -(x_fac - line.start()).dot(obs_unit_dir_perp);
          for (int i = 0; i < penetrations_k.size(); ++i) {
            penetrations_k[i] = dist - buffers[i];
          }
          if (!value_only) {
            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                -obs_unit_dir_perp;
            if (dist_to_rac_ != 0.0) {
              penetration_jacobians_k(PROB::kStateThetaIndex) =
                  -obs_unit_dir_perp.dot(normal * dist_to_rac_);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateThetaIndex) =
                  -obs_unit_dir_perp.dot(-tangent * dist_to_rac_);
            }
          }
        } else {
          // prod <=0.0
          const Vec2d diff_x1_fac = x_fac - line.start();
          const double diff_x1_fac_norm = diff_x1_fac.norm();
          const double diff_x1_fac_norm_div = 1.0 / diff_x1_fac_norm;
          const double sqrtqrdiff_x1 = Cube(diff_x1_fac_norm_div);
          for (int i = 0; i < penetrations_k.size(); ++i) {
            penetrations_k[i] = diff_x1_fac_norm - buffers[i];
          }
          if (!value_only) {
            penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
                diff_x1_fac * diff_x1_fac_norm_div;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex) =
                diff_x1_fac_norm_div -
                diff_x1_fac.x() * diff_x1_fac.x() * sqrtqrdiff_x1;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex) =
                -diff_x1_fac.x() * diff_x1_fac.y() * sqrtqrdiff_x1;
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex) =
                diff_x1_fac_norm_div -
                diff_x1_fac.y() * diff_x1_fac.y() * sqrtqrdiff_x1;
            if (dist_to_rac_ != 0.0) {
              penetration_jacobians_k(PROB::kStateThetaIndex) =
                  diff_x1_fac.dot(normal * dist_to_rac_) * diff_x1_fac_norm_div;
              penetration_hessians_k(PROB::kStateXIndex,
                                     PROB::kStateThetaIndex) =
                  normal.x() * dist_to_rac_ * diff_x1_fac_norm_div -
                  sqrtqrdiff_x1 * diff_x1_fac.x() *
                      diff_x1_fac.dot(normal * dist_to_rac_);
              penetration_hessians_k(PROB::kStateYIndex,
                                     PROB::kStateThetaIndex) =
                  normal.y() * dist_to_rac_ * diff_x1_fac_norm_div -
                  sqrtqrdiff_x1 * diff_x1_fac.y() *
                      diff_x1_fac.dot(normal * dist_to_rac_);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateXIndex) =
                  penetration_hessians_k(PROB::kStateXIndex,
                                         PROB::kStateThetaIndex);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateYIndex) =
                  penetration_hessians_k(PROB::kStateYIndex,
                                         PROB::kStateThetaIndex);
              penetration_hessians_k(PROB::kStateThetaIndex,
                                     PROB::kStateThetaIndex) =
                  (diff_x1_fac.dot(-tangent * dist_to_rac_) +
                   dist_to_rac_ * normal.dot(normal * dist_to_rac_)) *
                      diff_x1_fac_norm_div -
                  sqrtqrdiff_x1 * diff_x1_fac.dot(normal * dist_to_rac_) *
                      diff_x1_fac.dot(normal * dist_to_rac_);
            }
          }
        }
      }
    }
  }

 private:
  std::vector<Object> objects_;
  int num_objects_;

  double dist_to_rac_;
  double angle_to_axis_;

  std::vector<double> cascade_buffers_;
  const AvModelHelper<PROB> *av_model_helper_;

  std::vector<std::string> sub_names_;

  // States.
  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  std::vector<std::vector<double>> penetrations_;
  std::vector<PenetrationJacobianType> penetration_jacobians_;
  std::vector<PenetrationHessianType> penetration_hessians_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_PARTITIONED_OBSTACLE_COST_H_
