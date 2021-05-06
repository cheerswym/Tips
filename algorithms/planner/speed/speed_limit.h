#ifndef ONBOARD_PLANNER_SPEED_LIMIT_H_
#define ONBOARD_PLANNER_SPEED_LIMIT_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"
#include "onboard/lite/logging.h"

namespace qcraft::planner {

class SpeedLimit {
 public:
  struct SpeedLimitRange {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;
    std::string info;
  };

  SpeedLimit(const std::vector<SpeedLimitRange>& speed_limit_ranges,
             double default_speed_limit);

  double GetSpeedLimitByS(double s) const;

  std::optional<SpeedLimitRange> GetSpeedLimitRangeByS(double s) const;

  absl::Span<const SpeedLimitRange> merged_ranges() const {
    return merged_ranges_;
  }

 private:
  //    Origin speed limit ranges:
  //
  //    ^ speed_limit
  //    |               o----------------o
  //    |     o-------o
  //    |
  //    |          o----------o
  //    |
  //    |
  //    |------------------------------------->s
  //
  //    Merged speed limit ranges:
  //    (Merge the lower bound speed limit of each range.)
  //
  //    ^ speed_limit
  //    |                     o----------o
  //    |     o----o          |
  //    |          |          |
  //    |          o----------o
  //    |
  //    |
  //    |------------------------------------->s
  //
  std::vector<SpeedLimitRange> merged_ranges_;  // Sort by start_s.
  double default_speed_limit_;
};

// merged_range_indice
void MergeSpeedLimitRanges(
    absl::Span<const SpeedLimit::SpeedLimitRange> ranges,
    std::vector<SpeedLimit::SpeedLimitRange>* merged_ranges,
    std::vector<int>* merged_range_indices);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_LIMIT_H_
