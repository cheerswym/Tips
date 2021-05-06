#ifndef ONBOARD_PLANNER_COMMON_PATH_SL_BOUNDARY_H_
#define ONBOARD_PLANNER_COMMON_PATH_SL_BOUNDARY_H_

#include <utility>
#include <vector>

#include "onboard/math/vec.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft::planner {

class PathSlBoundary {
 public:
  PathSlBoundary() = default;

  explicit PathSlBoundary(
      std::vector<double> s, std::vector<double> ref_center_l,
      std::vector<double> right_l, std::vector<double> left_l,
      std::vector<double> target_right_l, std::vector<double> target_left_l,
      std::vector<Vec2d> ref_center_xy, std::vector<Vec2d> right_xy,
      std::vector<Vec2d> left_xy, std::vector<Vec2d> target_right_xy,
      std::vector<Vec2d> target_left_xy);

  // reference centers are calculated by boundary
  explicit PathSlBoundary(std::vector<double> s, std::vector<double> right_l,
                          std::vector<double> left_l,
                          std::vector<double> target_right_l,
                          std::vector<double> target_left_l,
                          std::vector<Vec2d> right_xy,
                          std::vector<Vec2d> left_xy,
                          std::vector<Vec2d> target_right_xy,
                          std::vector<Vec2d> target_left_xy);

  bool IsEmpty() const { return s_vec_.empty(); }
  int size() const { return s_vec_.size(); }

  absl::Span<const double> s_vector() const { return s_vec_; }

  double end_s() const {
    QCHECK(!s_vec_.empty()) << "Empty path sl boundary!";
    return s_vec_.back();
  }

  absl::Span<const double> reference_center_l_vector() const {
    return ref_center_l_vec_;
  }

  absl::Span<const double> right_l_vector() const { return right_l_vec_; }

  absl::Span<const double> left_l_vector() const { return left_l_vec_; }

  absl::Span<const double> target_right_l_vector() const {
    return target_right_l_vec_;
  }

  absl::Span<const double> target_left_l_vector() const {
    return target_left_l_vec_;
  }

  absl::Span<const Vec2d> reference_center_xy_vector() const {
    return ref_center_xy_vec_;
  }

  absl::Span<const Vec2d> right_xy_vector() const { return right_xy_vec_; }

  absl::Span<const Vec2d> left_xy_vector() const { return left_xy_vec_; }

  absl::Span<const Vec2d> target_right_xy_vector() const {
    return target_right_xy_vec_;
  }

  absl::Span<const Vec2d> target_left_xy_vector() const {
    return target_left_xy_vec_;
  }

  // {right_l, left_l}
  std::pair<double, double> QueryBoundaryL(double s) const;
  // {target_right_l, target_left_l}
  std::pair<double, double> QueryTargetBoundaryL(double s) const;
  // {right_xy, left_xy}
  std::pair<Vec2d, Vec2d> QueryBoundaryXY(double s) const;
  // {target_right_xy, target_left_xy}
  std::pair<Vec2d, Vec2d> QueryTargetBoundaryXY(double s) const;

  double QueryReferenceCenterL(double s) const;
  Vec2d QueryReferenceCenterXY(double s) const;

 private:
  // {start_index, lerp_factor}
  std::pair<int, double> FindLerpInfo(double s) const;

  std::vector<double> s_vec_;             // Station.
  std::vector<double> ref_center_l_vec_;  // Reference center
  // Right or left l vec is always the outer boundary
  std::vector<double> right_l_vec_;
  std::vector<double> left_l_vec_;
  // Target right or left l vec is always the inner boundary
  std::vector<double> target_right_l_vec_;
  std::vector<double> target_left_l_vec_;
  std::vector<Vec2d> ref_center_xy_vec_;
  std::vector<Vec2d> right_xy_vec_;
  std::vector<Vec2d> left_xy_vec_;
  std::vector<Vec2d> target_right_xy_vec_;
  std::vector<Vec2d> target_left_xy_vec_;
};

inline std::pair<double, double> PathSlBoundary::QueryBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(
      Lerp(right_l_vec_[index], right_l_vec_[index + 1], factor),
      Lerp(left_l_vec_[index], left_l_vec_[index + 1], factor));
}

inline std::pair<Vec2d, Vec2d> PathSlBoundary::QueryBoundaryXY(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(
      Lerp(right_xy_vec_[index], right_xy_vec_[index + 1], factor),
      Lerp(left_xy_vec_[index], left_xy_vec_[index + 1], factor));
}

inline std::pair<double, double> PathSlBoundary::QueryTargetBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(
      Lerp(target_right_l_vec_[index], target_right_l_vec_[index + 1], factor),
      Lerp(target_left_l_vec_[index], target_left_l_vec_[index + 1], factor));
}

inline std::pair<Vec2d, Vec2d> PathSlBoundary::QueryTargetBoundaryXY(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(
      Lerp(target_right_xy_vec_[index], target_right_xy_vec_[index + 1],
           factor),
      Lerp(target_left_xy_vec_[index], target_left_xy_vec_[index + 1], factor));
}

inline double PathSlBoundary::QueryReferenceCenterL(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return Lerp(ref_center_l_vec_[index], ref_center_l_vec_[index + 1], factor);
}

inline Vec2d PathSlBoundary::QueryReferenceCenterXY(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return Lerp(ref_center_xy_vec_[index], ref_center_xy_vec_[index + 1], factor);
}

inline std::pair<int, double> PathSlBoundary::FindLerpInfo(double s) const {
  if (s <= s_vec_.front()) {
    return {0, (s - s_vec_[0]) / (s_vec_[1] - s_vec_[0])};
  }

  auto succ_it = std::upper_bound(s_vec_.begin(), s_vec_.end(), s);
  if (succ_it == s_vec_.end()) succ_it = std::prev(succ_it);

  const auto prev_it = std::prev(succ_it);
  return {prev_it - s_vec_.begin(), (s - *prev_it) / (*succ_it - *prev_it)};
}

}  // namespace qcraft::planner

#endif
