#ifndef ONBOARD_CONTROL_CONTROLLERS_CONTROL_TRACKING_STATISTICS_H_
#define ONBOARD_CONTROL_CONTROLLERS_CONTROL_TRACKING_STATISTICS_H_

#include <cmath>
#include <deque>

#include "absl/time/time.h"
#include "glog/logging.h"
#include "onboard/lite/logging.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {

class ControlTrackingStatistics {
 public:
  // Computes the error in a time sliding window.
  explicit ControlTrackingStatistics(absl::Duration monitor_duration)
      : monitor_duration_(monitor_duration) {}

  // Takes an error record at `now`. It removes items that are older than `now -
  // monitor_duration_`, and add the new error data in `mpc_debug` into its
  // memory.
  void Process(absl::Time now, const SimpleMPCDebug &mpc_debug);

  // Returns the number of records stored.
  int size() const { return error_records_.size(); }

  // Returns the statistics of lateral errors.
  TrackingErrorStat ComputeLateralErrorStats() const;

  // Returns the statistics of station errors.
  TrackingErrorStat ComputeStationErrorStats() const;

 private:
  struct ErrorRecord {
    double lateral_error;
    double station_error;
    absl::Time time;
  };
  absl::Duration monitor_duration_;
  std::deque<ErrorRecord> error_records_;
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CONTROLLERS_CONTROL_TRACKING_STATISTICS_H_
