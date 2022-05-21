#include "onboard/utils/time_util.h"

#include <functional>
#include <string>

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(TimeUtilTest, TimeConversion) {
  absl::Time now = absl::Now();
  double double_now = ToUnixDoubleSeconds(now);
  absl::Time converted_now = FromUnixDoubleSeconds(double_now);
  auto duration = now - converted_now;
  if (duration < absl::ZeroDuration()) {
    duration = -duration;
  }
  EXPECT_LE(duration, absl::Microseconds(1));
}

TEST(TimeUtilTest, TimeProtoConversion) {
  auto tp = std::chrono::system_clock::from_time_t(123);
  absl::Time t = absl::FromChrono(tp);
  google::protobuf::Timestamp proto;
  ToProto(t, &proto);
  absl::Time t1 = FromProto(proto);
  EXPECT_EQ(t, t1);
}

}  // namespace
}  // namespace qcraft
