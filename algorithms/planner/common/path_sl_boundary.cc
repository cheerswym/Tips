#include "onboard/planner/common/path_sl_boundary.h"

#include <algorithm>

#include "onboard/math/util.h"

namespace qcraft::planner {

PathSlBoundary::PathSlBoundary(
    std::vector<double> s, std::vector<double> ref_center_l,
    std::vector<double> right_l, std::vector<double> left_l,
    std::vector<double> target_right_l, std::vector<double> target_left_l,
    std::vector<Vec2d> ref_center_xy, std::vector<Vec2d> right_xy,
    std::vector<Vec2d> left_xy, std::vector<Vec2d> target_right_xy,
    std::vector<Vec2d> target_left_xy)
    : s_vec_(std::move(s)),
      ref_center_l_vec_(std::move(ref_center_l)),
      right_l_vec_(std::move(right_l)),
      left_l_vec_(std::move(left_l)),
      target_right_l_vec_(std::move(target_right_l)),
      target_left_l_vec_(std::move(target_left_l)),
      ref_center_xy_vec_(std::move(ref_center_xy)),
      right_xy_vec_(std::move(right_xy)),
      left_xy_vec_(std::move(left_xy)),
      target_right_xy_vec_(std::move(target_right_xy)),
      target_left_xy_vec_(std::move(target_left_xy)) {
  const int vec_size = s_vec_.size();
  QCHECK_GT(vec_size, 1);
  QCHECK_EQ(vec_size, ref_center_l_vec_.size());
  QCHECK_EQ(vec_size, ref_center_xy_vec_.size());
  QCHECK_EQ(vec_size, right_l_vec_.size());
  QCHECK_EQ(vec_size, left_l_vec_.size());
  QCHECK_EQ(vec_size, target_right_l_vec_.size());
  QCHECK_EQ(vec_size, target_left_l_vec_.size());
  QCHECK_EQ(vec_size, right_xy_vec_.size());
  QCHECK_EQ(vec_size, left_xy_vec_.size());
  QCHECK_EQ(vec_size, target_right_xy_vec_.size());
  QCHECK_EQ(vec_size, target_left_xy_vec_.size());
}

PathSlBoundary::PathSlBoundary(
    std::vector<double> s, std::vector<double> right_l,
    std::vector<double> left_l, std::vector<double> target_right_l,
    std::vector<double> target_left_l, std::vector<Vec2d> right_xy,
    std::vector<Vec2d> left_xy, std::vector<Vec2d> target_right_xy,
    std::vector<Vec2d> target_left_xy)
    : s_vec_(std::move(s)),
      right_l_vec_(std::move(right_l)),
      left_l_vec_(std::move(left_l)),
      target_right_l_vec_(std::move(target_right_l)),
      target_left_l_vec_(std::move(target_left_l)),
      right_xy_vec_(std::move(right_xy)),
      left_xy_vec_(std::move(left_xy)),
      target_right_xy_vec_(std::move(target_right_xy)),
      target_left_xy_vec_(std::move(target_left_xy)) {
  const int vec_size = s_vec_.size();
  QCHECK_GT(vec_size, 1);
  QCHECK_EQ(vec_size, right_l_vec_.size());
  QCHECK_EQ(vec_size, left_l_vec_.size());
  QCHECK_EQ(vec_size, target_right_l_vec_.size());
  QCHECK_EQ(vec_size, target_left_l_vec_.size());
  QCHECK_EQ(vec_size, right_xy_vec_.size());
  QCHECK_EQ(vec_size, left_xy_vec_.size());
  QCHECK_EQ(vec_size, target_right_xy_vec_.size());
  QCHECK_EQ(vec_size, target_left_xy_vec_.size());

  ref_center_l_vec_.reserve(vec_size);
  ref_center_xy_vec_.reserve(vec_size);
  for (int i = 0; i < vec_size; ++i) {
    ref_center_l_vec_.emplace_back(0.5 * (right_l_vec_[i] + left_l_vec_[i]));
    ref_center_xy_vec_.emplace_back(0.5 * (right_xy_vec_[i], left_xy_vec_[i]));
  }
}

}  // namespace qcraft::planner
