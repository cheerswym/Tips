#include "onboard/control/controllers/control_tracking_statistics.h"

#include "absl/time/time.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "onboard/utils/test_util.h"

namespace qcraft {
namespace control {
namespace {

SimpleMPCDebug MakeErrorMsg(double station_error, double lateral_error) {
  SimpleMPCDebug mpc_debug;
  mpc_debug.set_station_error(station_error);
  mpc_debug.set_lateral_error(lateral_error);
  return mpc_debug;
}

TEST(ControlTrackingStatistics, History) {
  ControlTrackingStatistics error_stats(absl::Seconds(1.0));
  absl::Time t = absl::UnixEpoch();
  EXPECT_EQ(error_stats.size(), 0);
  error_stats.Process(t, MakeErrorMsg(0.0, 0.0));
  EXPECT_EQ(error_stats.size(), 1);
  const absl::Duration delta_t = absl::Seconds(0.6);
  error_stats.Process(t + delta_t, MakeErrorMsg(0.0, 0.0));
  EXPECT_EQ(error_stats.size(), 2);
  error_stats.Process(t + 2 * delta_t, MakeErrorMsg(0.0, 0.0));
  EXPECT_EQ(error_stats.size(), 2);  // The number should not increase.
}

TEST(ControlTrackingStatistics, ErrorStats) {
  ControlTrackingStatistics error_stats(absl::Seconds(1.0));
  const absl::Time t = absl::UnixEpoch();
  EXPECT_EQ(error_stats.size(), 0);
  error_stats.Process(
      t, MakeErrorMsg(/*station_error=*/-2.0, /*lateral_error=*/0.0));
  error_stats.Process(
      t + absl::Seconds(0.1),
      MakeErrorMsg(/*station_error=*/1.0, /*lateral_error=*/2.0));
  error_stats.Process(
      t + absl::Seconds(0.2),
      MakeErrorMsg(/*station_error=*/0.0, /*lateral_error=*/-1.0));
  EXPECT_EQ(error_stats.size(), 3);
  EXPECT_THAT(error_stats.ComputeStationErrorStats(),
              ProtoPartialApproximatelyMatchesText(R"(
                                            max_error_relative_time: -0.2
                                            max: -2.0
                                            mean: -0.333
                                            rms: 1.29
                                            )",
                                                   1e-3));
  EXPECT_THAT(error_stats.ComputeLateralErrorStats(),
              ProtoPartialApproximatelyMatchesText(R"(
                                            max_error_relative_time: -0.1
                                            max: 2
                                            mean: 0.333
                                            rms: 1.29
                                            )",
                                                   1e-3));
}

}  // namespace
}  // namespace control
}  // namespace qcraft
