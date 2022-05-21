#include "onboard/control/controllers/control_tracking_statistics.h"

#include <utility>

#include "onboard/math/util.h"

namespace qcraft {
namespace control {

// Compute error statistics from a container `RECORDS` with `ERROR_TYPE`. The
// result will be saved to `STATS`. Each element in `RECORDS` should have an
// ERROR_TYPE field.
#define COLLECT_ERROR_STATS(RECORDS, ERROR_TYPE, STATS)           \
  do {                                                            \
    double max_abs_error = 0.0;                                   \
    double max_signed_error = 0.0;                                \
    double sum = 0.0;                                             \
    double sqr_sum = 0.0;                                         \
    absl::Duration max_error_relative_time;                       \
    absl::Time latest_time = RECORDS.back().time;                 \
    for (const auto &error_point : RECORDS) {                     \
      const auto error = error_point.ERROR_TYPE;                  \
      if (std::abs(error) > max_abs_error) {                      \
        max_abs_error = std::abs(error);                          \
        max_signed_error = error;                                 \
        max_error_relative_time = error_point.time - latest_time; \
      }                                                           \
      sum += error;                                               \
      sqr_sum += Sqr(error);                                      \
    }                                                             \
    const int num_records = RECORDS.size();                       \
    STATS.set_mean(sum / num_records);                            \
    STATS.set_max(max_signed_error);                              \
    STATS.set_rms(std::sqrt(sqr_sum / num_records));              \
    STATS.set_max_error_relative_time(                            \
        absl::ToDoubleSeconds(max_error_relative_time));          \
  } while (false)

void ControlTrackingStatistics::Process(absl::Time now,
                                        const SimpleMPCDebug &mpc_debug) {
  while (!error_records_.empty() &&
         now - error_records_.front().time > monitor_duration_) {
    error_records_.pop_front();
  }
  error_records_.push_back({.lateral_error = mpc_debug.lateral_error(),
                            .station_error = mpc_debug.station_error(),
                            .time = now});
}

TrackingErrorStat ControlTrackingStatistics::ComputeStationErrorStats() const {
  QCHECK_GT(size(), 0);
  TrackingErrorStat error_stats;
  COLLECT_ERROR_STATS(error_records_, station_error, error_stats);
  return error_stats;
}

TrackingErrorStat ControlTrackingStatistics::ComputeLateralErrorStats() const {
  QCHECK_GT(size(), 0);
  TrackingErrorStat error_stats;
  COLLECT_ERROR_STATS(error_records_, lateral_error, error_stats);
  return error_stats;
}

}  // namespace control
}  // namespace qcraft
