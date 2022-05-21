#ifndef ONBOARD_PREDICTION_CONFLICT_RESOLVER_SVT_FEATURE_COST_H
#define ONBOARD_PREDICTION_CONFLICT_RESOLVER_SVT_FEATURE_COST_H

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "onboard/planner/speed/speed_vector.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/prediction/conflict_resolver/object_svt_sample.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace prediction {
class SvtFeatureCost {
 public:
  explicit SvtFeatureCost(std::string name) : name_(std::move(name)) {}

  virtual void ComputeFeatureCost(const std::vector<SvtState>& states,
                                  absl::Span<double> cost) const = 0;

  absl::string_view name() const { return name_; }
  virtual ~SvtFeatureCost() {}

 private:
  std::string name_;
  // TODO(changqing): Add feature size.
};

class SvtObjectCollisionFeatureCost : public SvtFeatureCost {
 public:
  explicit SvtObjectCollisionFeatureCost(
      std::vector<double> stationary_objects,
      std::vector<const planner::StBoundary*> moving_objects,
      double stationary_follow_distance, double dynamic_follow_distance)
      : SvtFeatureCost("svt_object_collision"),
        stationary_follow_distance_(stationary_follow_distance),
        dynamic_follow_distance_(dynamic_follow_distance),
        stationary_objects_(stationary_objects),
        moving_objects_(moving_objects) {
    if (!stationary_objects.empty()) {
      min_s_ = stationary_objects[0];
    }
    inv_dynamic_follow_distance_ = 1.0 / dynamic_follow_distance_;
    inv_stationary_follow_distance_ = 1.0 / stationary_follow_distance_;
  }

  void ComputeFeatureCost(const std::vector<SvtState>& states,
                          absl::Span<double> cost) const override;

 private:
  double stationary_follow_distance_ = 1e-6;
  double dynamic_follow_distance_ = 1e-6;
  // Currently consider the closest stationary object.
  std::vector<double> stationary_objects_;
  std::vector<const planner::StBoundary*> moving_objects_;
  double min_s_ = std::numeric_limits<double>::max();
  double inv_dynamic_follow_distance_ = 0.0;
  double inv_stationary_follow_distance_ = 0.0;
};

class SvtStoplineFeatureCost : public SvtFeatureCost {
 public:
  explicit SvtStoplineFeatureCost(const std::vector<double>& stoplines)
      : SvtFeatureCost("svt_stopline"), stoplines_(stoplines) {}

  void ComputeFeatureCost(const std::vector<SvtState>& states,
                          absl::Span<double> cost) const override;

 private:
  std::vector<double> stoplines_;
};

class SvtComfortFeatureCost : public SvtFeatureCost {
 public:
  SvtComfortFeatureCost() : SvtFeatureCost("svt_comfort") {}

  void ComputeFeatureCost(const std::vector<SvtState>& states,
                          absl::Span<double> cost) const override;
};

class SvtReferenceSpeedFeatureCost : public SvtFeatureCost {
 public:
  explicit SvtReferenceSpeedFeatureCost(const planner::SpeedVector* ref_speed)
      : SvtFeatureCost("svt_reference_speed"), ref_speed_(ref_speed) {
    total_length_ = ref_speed_->TotalLength();
    total_time_ = ref_speed_->TotalTime();
    inv_average_v_ = total_time_ / total_length_;
    inv_length_ = 1.0 / total_length_;
  }

  void ComputeFeatureCost(const std::vector<SvtState>& states,
                          absl::Span<double> cost) const override;

 private:
  const planner::SpeedVector* ref_speed_;
  double total_length_ = 0.0;
  double total_time_ = 0.0;
  double inv_average_v_ = 0.0;
  double inv_length_ = 0.0;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_CONFLICT_RESOLVER_SVT_FEATURE_COST_H
