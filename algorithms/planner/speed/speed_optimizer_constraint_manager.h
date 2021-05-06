#ifndef ONBOARD_PLANNER_SPEED_OPTIMIZER_CONSTRAINT_MANAGER_H_
#define ONBOARD_PLANNER_SPEED_OPTIMIZER_CONSTRAINT_MANAGER_H_

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/strings/str_format.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {

/**
  This class manages the constraint data of speed optimizer.
*/
class SpeedOptimizerConstraintManager {
 public:
  using Sfp = SpeedFinderParamsProto;

  enum class BoundDirType {
    UPPER = 0,
    LOWER = 1,
  };

  enum class BoundStrType {
    WEAK = 0,
    MEDIUM = 1,  // Used by stationary object.
    STRONG = 2,
  };

  struct SoftConstraintInput {
    int knot_idx = 0;
    double weight = 0.0;
    double bound = 0.0;
  };

  struct SoftConstraint {
    int knot_idx = 0;
    int slack_idx = 0;
    double bound = 0.0;
  };

  struct HardConstraint {
    int knot_idx = 0;
    double lower_bound = 0.0;
    double upper_bound = 0.0;
  };

  struct SlackWeight {
    int slack_idx = 0;
    double weight = 0.0;
  };

  // TODO(ping): Simplify this data structure.
  struct SSoftConstraintData {
    // This map will contain the slack_idx. time_range_idx as its key, and the
    // map of strength_type to slack_idx as the value.
    absl::flat_hash_map<int, absl::flat_hash_map<BoundStrType, int>>
        time_range_idx_to_slack_idx_map;

    // This map will contain the soft_constraint. knot_idx as its key, and the
    // map of strength_type to soft_constraint as the value.
    absl::flat_hash_map<int, absl::flat_hash_map<BoundStrType, SoftConstraint>>
        soft_constraint_map;

    // This map will contain the slack_weight. slack_idx as its key, and the
    // map of knot_idx to slack_weight as the value.
    absl::flat_hash_map<int, absl::flat_hash_map<int, double>> slack_weight_map;
  };

  struct SpeedSoftConstraintData {
    // This map will contain the slack_idx. speed_limit_type as its key, and the
    // slack_idx as the value.
    absl::flat_hash_map<Sfp::SpeedLimitType, int> type_to_slack_idx_map;

    std::vector<SoftConstraint> soft_constraints;

    // This map will contain the slack_weight. slack_idx as its key, and the
    // slack_weight as the value.
    absl::flat_hash_map<int, double> slack_weight_map;
  };

  struct AccelSoftConstraintData {
    std::vector<SoftConstraint> soft_constraints;
    absl::flat_hash_map<int, double> slack_weight_map;
  };

  SpeedOptimizerConstraintManager(std::vector<double> piecewise_time_range,
                                  double delta_t);

  void AddSoftSLowerConstraint(BoundStrType strength_type,
                               const SoftConstraintInput &constraint_input);

  void AddSoftSUpperConstraint(BoundStrType strength_type,
                               const SoftConstraintInput &constraint_input);

  void AddHardSConstraint(int knot_idx, double lower_bound, double upper_bound);

  void AddSoftSpeedLowerConstraints(
      const Sfp::SpeedLimitType &speed_limit_type,
      const SoftConstraintInput &constraint_input);

  void AddSoftSpeedUpperConstraints(
      const Sfp::SpeedLimitType &speed_limit_type,
      const SoftConstraintInput &constraint_input);

  void AddAccelConstraint(int knot_idx, double lower_bound, double upper_bound);

  void AddAccelSoftUpperConstraint(const SoftConstraintInput &accel_constraint);

  void AddAccelSoftLowerConstraint(const SoftConstraintInput &accel_constraint);

  std::vector<SoftConstraint> GetLowerConstraintOfSoftS() const;

  std::vector<SoftConstraint> GetUpperConstraintOfSoftS() const;

  absl::Span<const SoftConstraint> GetLowerConstraintOfSpeed() const;

  absl::Span<const SoftConstraint> GetUpperConstraintOfSpeed() const;

  absl::Span<const SoftConstraint> GetLowerConstraintOfSoftAccel() const {
    return accel_lower_data_.soft_constraints;
  }

  absl::Span<const SoftConstraint> GetUpperConstraintOfSoftAccel() const {
    return accel_upper_data_.soft_constraints;
  }

  absl::Span<const HardConstraint> hard_s_constraint() const {
    return hard_s_constraint_;
  }

  std::vector<SlackWeight> GetLowerSlackWeightOfSoftS() const;

  std::vector<SlackWeight> GetUpperSlackWeightOfSoftS() const;

  std::vector<SlackWeight> GetLowerSlackWeightOfSpeed() const;

  std::vector<SlackWeight> GetUpperSlackWeightOfSpeed() const;

  const absl::flat_hash_map<int, double> &GetLowerSlackWeightOfAccel() const {
    return accel_lower_data_.slack_weight_map;
  }

  const absl::flat_hash_map<int, double> &GetUpperSlackWeightOfAccel() const {
    return accel_upper_data_.slack_weight_map;
  }

  absl::Span<const HardConstraint> hard_a_constraint() const {
    return hard_a_constraint_;
  }

  int GetSlackNumOfLowerS() const {
    return s_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperS() const {
    return s_upper_data_.slack_weight_map.size();
  }

  int GetSlackNumOfLowerSpeed() const {
    return speed_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperSpeed() const {
    return speed_upper_data_.slack_weight_map.size();
  }

  int GetSlackNumOfLowerAccel() const {
    return accel_lower_data_.slack_weight_map.size();
  }

  int GetSlackNumOfUpperAccel() const {
    return accel_upper_data_.slack_weight_map.size();
  }

 private:
  // Convert knot_idx to time_range_idx.
  int GetTimeRangeIndex(int knot_idx) const;

  // Return the knot number of time_range_idx.
  int GetKnotNum(int time_range_idx) const;

  // This vector contains a series of monotonically increasing discrete time
  // points. For example, {0.0, 5.0, 10.0}, which means that we divide the time
  // axis into two segments. The first segment is 0.0-5.0 seconds and the second
  // segment is 5.0-10.0 seconds.
  std::vector<double> piecewise_time_range_;
  double delta_t_ = 0.0;

  int s_lower_slack_idx_ = 0;
  int s_upper_slack_idx_ = 0;
  int v_lower_slack_idx_ = 0;
  int v_upper_slack_idx_ = 0;

  SSoftConstraintData s_upper_data_;
  SSoftConstraintData s_lower_data_;
  std::vector<HardConstraint> hard_s_constraint_;

  SpeedSoftConstraintData speed_upper_data_;
  SpeedSoftConstraintData speed_lower_data_;

  AccelSoftConstraintData accel_upper_data_;
  AccelSoftConstraintData accel_lower_data_;

  std::vector<HardConstraint> hard_a_constraint_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_OPTIMIZER_CONSTRAINT_MANAGER_H_
