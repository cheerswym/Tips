#include "onboard/planner/speed/speed_optimizer_constraint_manager.h"

#include "absl/strings/str_format.h"

namespace qcraft::planner {
namespace {
using Sfp = SpeedFinderParamsProto;
using BoundStrType = SpeedOptimizerConstraintManager::BoundStrType;
using BoundDirType = SpeedOptimizerConstraintManager::BoundDirType;
using SoftConstraint = SpeedOptimizerConstraintManager::SoftConstraint;
using SoftConstraintInput =
    SpeedOptimizerConstraintManager::SoftConstraintInput;
using SSoftConstraintData =
    SpeedOptimizerConstraintManager::SSoftConstraintData;
using SpeedSoftConstraintData =
    SpeedOptimizerConstraintManager::SpeedSoftConstraintData;
using SlackWeight = SpeedOptimizerConstraintManager::SlackWeight;

void UpdateSlackIndexOfSoftSConstraint(
    int time_range_idx, BoundStrType strength_type,
    absl::flat_hash_map<int, absl::flat_hash_map<BoundStrType, int>>
        *slack_idx_map,
    int *slack_idx) {
  QCHECK_NOTNULL(slack_idx_map);
  QCHECK_NOTNULL(slack_idx);
  if (!slack_idx_map->contains(time_range_idx) ||
      !(*slack_idx_map)[time_range_idx].contains(strength_type)) {
    (*slack_idx_map)[time_range_idx][strength_type] = (*slack_idx)++;
  }
}

void UpdateSlackIndexOfSoftSpeedConstraint(
    const Sfp::SpeedLimitType &speed_limit_type,
    absl::flat_hash_map<Sfp::SpeedLimitType, int> *slack_idx_map,
    int *slack_idx) {
  QCHECK_NOTNULL(slack_idx_map);
  QCHECK_NOTNULL(slack_idx);
  if (!slack_idx_map->contains(speed_limit_type)) {
    (*slack_idx_map)[speed_limit_type] = (*slack_idx)++;
  }
}

// This function updates the constraint in two cases:
// Case 1: The constraint with strength_type at time_range_idx position
//         has not been added.
// Case 2: The newly added constraint bound is less(greater) than
//         the bound of added constraint.
void UpdateSoftSConstraint(int time_range_idx, int knot_num,
                           BoundStrType strength_type, BoundDirType dir_type,
                           const SoftConstraintInput &constraint_input,
                           SSoftConstraintData *constraint_data) {
  QCHECK_NOTNULL(constraint_data);

  const auto &slack_idx_map = constraint_data->time_range_idx_to_slack_idx_map;
  auto &constraint_map = constraint_data->soft_constraint_map;
  auto &weight_map = constraint_data->slack_weight_map;

  QCHECK(slack_idx_map.contains(time_range_idx));
  QCHECK(FindOrDie(slack_idx_map, time_range_idx).contains(strength_type));

  const int knot_idx = constraint_input.knot_idx;
  const int slack_idx = slack_idx_map.at(time_range_idx).at(strength_type);

  bool is_update = false;

  if (!constraint_map.contains(knot_idx) ||
      !constraint_map[knot_idx].contains(strength_type)) {
    is_update = true;
  }

  if (!is_update) {
    const double raw_bound = constraint_map[knot_idx][strength_type].bound;
    if (dir_type == BoundDirType::UPPER ? constraint_input.bound < raw_bound
                                        : constraint_input.bound > raw_bound) {
      is_update = true;
    }
  }

  if (is_update) {
    constraint_map[knot_idx][strength_type] = {.knot_idx = knot_idx,
                                               .slack_idx = slack_idx,
                                               .bound = constraint_input.bound};
    weight_map[slack_idx][knot_idx] = constraint_input.weight * knot_num;
  }
}

void UpdateSoftSpeedConstraint(const Sfp::SpeedLimitType &speed_limit_type,
                               const SoftConstraintInput &constraint_input,
                               SpeedSoftConstraintData *constraint_data) {
  QCHECK_NOTNULL(constraint_data);
  const int slack_index =
      FindOrDie(constraint_data->type_to_slack_idx_map, speed_limit_type);
  constraint_data->soft_constraints.push_back(
      {.knot_idx = constraint_input.knot_idx,
       .slack_idx = slack_index,
       .bound = constraint_input.bound});
  constraint_data->slack_weight_map[slack_index] += constraint_input.weight;
}

std::vector<SlackWeight> AssembleSlackWeightOfS(
    const absl::flat_hash_map<int, absl::flat_hash_map<int, double>>
        &slack_weight_map) {
  std::vector<SlackWeight> slack_weights;
  slack_weights.reserve(slack_weight_map.size());
  for (const auto &[slack_idx, weights] : slack_weight_map) {
    const int weight_size = weights.size();
    double average_weight = 0.0;
    for (const auto &[_, weight] : weights) {
      average_weight += weight;
    }
    QCHECK_GT(weight_size, 0);
    average_weight /= weight_size;
    slack_weights.push_back({.slack_idx = slack_idx, .weight = average_weight});
  }
  return slack_weights;
}

std::vector<SlackWeight> AssembleSlackWeightOfSpeed(
    const absl::flat_hash_map<int, double> &slack_weight_map) {
  std::vector<SlackWeight> slack_weights;
  slack_weights.reserve(slack_weight_map.size());
  for (const auto [slack_idx, weight] : slack_weight_map) {
    slack_weights.push_back({.slack_idx = slack_idx, .weight = weight});
  }
  return slack_weights;
}

}  // namespace

SpeedOptimizerConstraintManager::SpeedOptimizerConstraintManager(
    std::vector<double> piecewise_time_range, double delta_t)
    : piecewise_time_range_(std::move(piecewise_time_range)),
      delta_t_(delta_t) {
  QCHECK_GE(piecewise_time_range_.size(), 2);
}

void SpeedOptimizerConstraintManager::AddSoftSUpperConstraint(
    BoundStrType strength_type, const SoftConstraintInput &constraint_input) {
  const int time_range_idx = GetTimeRangeIndex(constraint_input.knot_idx);
  const int knot_num = GetKnotNum(time_range_idx);
  UpdateSlackIndexOfSoftSConstraint(
      time_range_idx, strength_type,
      &s_upper_data_.time_range_idx_to_slack_idx_map, &s_upper_slack_idx_);
  UpdateSoftSConstraint(time_range_idx, knot_num, strength_type,
                        BoundDirType::UPPER, constraint_input, &s_upper_data_);
}

void SpeedOptimizerConstraintManager::AddSoftSLowerConstraint(
    BoundStrType strength_type, const SoftConstraintInput &constraint_input) {
  const int time_range_idx = GetTimeRangeIndex(constraint_input.knot_idx);
  const int knot_num = GetKnotNum(time_range_idx);
  UpdateSlackIndexOfSoftSConstraint(
      time_range_idx, strength_type,
      &s_lower_data_.time_range_idx_to_slack_idx_map, &s_lower_slack_idx_);
  UpdateSoftSConstraint(time_range_idx, knot_num, strength_type,
                        BoundDirType::LOWER, constraint_input, &s_lower_data_);
}

void SpeedOptimizerConstraintManager::AddHardSConstraint(int knot_idx,
                                                         double lower_bound,
                                                         double upper_bound) {
  hard_s_constraint_.push_back({.knot_idx = knot_idx,
                                .lower_bound = lower_bound,
                                .upper_bound = upper_bound});
}

std::vector<SoftConstraint>
SpeedOptimizerConstraintManager::GetLowerConstraintOfSoftS() const {
  std::vector<SoftConstraint> constraints;
  for (const auto &[_, strength_map] : s_lower_data_.soft_constraint_map) {
    for (const auto &[_, constraint] : strength_map) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

std::vector<SoftConstraint>
SpeedOptimizerConstraintManager::GetUpperConstraintOfSoftS() const {
  std::vector<SoftConstraint> constraints;
  for (const auto &[_, strength_map] : s_upper_data_.soft_constraint_map) {
    for (const auto &[_, constraint] : strength_map) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

void SpeedOptimizerConstraintManager::AddSoftSpeedLowerConstraints(
    const Sfp::SpeedLimitType &speed_limit_type,
    const SoftConstraintInput &constraint_input) {
  UpdateSlackIndexOfSoftSpeedConstraint(
      speed_limit_type, &speed_lower_data_.type_to_slack_idx_map,
      &v_lower_slack_idx_);
  UpdateSoftSpeedConstraint(speed_limit_type, constraint_input,
                            &speed_lower_data_);
}

void SpeedOptimizerConstraintManager::AddSoftSpeedUpperConstraints(
    const Sfp::SpeedLimitType &speed_limit_type,
    const SoftConstraintInput &constraint_input) {
  UpdateSlackIndexOfSoftSpeedConstraint(
      speed_limit_type, &speed_upper_data_.type_to_slack_idx_map,
      &v_upper_slack_idx_);
  UpdateSoftSpeedConstraint(speed_limit_type, constraint_input,
                            &speed_upper_data_);
}

void SpeedOptimizerConstraintManager::AddAccelConstraint(int knot_idx,
                                                         double lower_bound,
                                                         double upper_bound) {
  hard_a_constraint_.push_back({.knot_idx = knot_idx,
                                .lower_bound = lower_bound,
                                .upper_bound = upper_bound});
}

void SpeedOptimizerConstraintManager::AddAccelSoftUpperConstraint(
    const SoftConstraintInput &accel_constraint) {
  accel_upper_data_.soft_constraints.push_back(
      {.knot_idx = accel_constraint.knot_idx,
       .slack_idx = 0,
       .bound = accel_constraint.bound});
  accel_upper_data_.slack_weight_map[0] += accel_constraint.weight;
}

void SpeedOptimizerConstraintManager::AddAccelSoftLowerConstraint(
    const SoftConstraintInput &accel_constraint) {
  accel_lower_data_.soft_constraints.push_back(
      {.knot_idx = accel_constraint.knot_idx,
       .slack_idx = 0,
       .bound = accel_constraint.bound});
  accel_lower_data_.slack_weight_map[0] += accel_constraint.weight;
}

absl::Span<const SoftConstraint>
SpeedOptimizerConstraintManager::GetLowerConstraintOfSpeed() const {
  return speed_lower_data_.soft_constraints;
}

absl::Span<const SoftConstraint>
SpeedOptimizerConstraintManager::GetUpperConstraintOfSpeed() const {
  return speed_upper_data_.soft_constraints;
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetLowerSlackWeightOfSoftS() const {
  return AssembleSlackWeightOfS(s_lower_data_.slack_weight_map);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetUpperSlackWeightOfSoftS() const {
  return AssembleSlackWeightOfS(s_upper_data_.slack_weight_map);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetLowerSlackWeightOfSpeed() const {
  return AssembleSlackWeightOfSpeed(speed_lower_data_.slack_weight_map);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetUpperSlackWeightOfSpeed() const {
  return AssembleSlackWeightOfSpeed(speed_upper_data_.slack_weight_map);
}

int SpeedOptimizerConstraintManager::GetTimeRangeIndex(int knot_idx) const {
  const double time = knot_idx * delta_t_;
  const int dist =
      std::distance(piecewise_time_range_.begin(),
                    std::lower_bound(piecewise_time_range_.begin(),
                                     piecewise_time_range_.end(), time));
  return std::clamp(dist - 1, 0,
                    static_cast<int>(piecewise_time_range_.size() - 2));
}

int SpeedOptimizerConstraintManager::GetKnotNum(int time_range_idx) const {
  return (piecewise_time_range_[time_range_idx + 1] -
          piecewise_time_range_[time_range_idx]) /
         delta_t_;
}

}  // namespace qcraft::planner
