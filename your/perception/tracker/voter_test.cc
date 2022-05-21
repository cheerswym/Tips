#include "onboard/perception/tracker/voter.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/math/util.h"

namespace qcraft::tracker {

TEST(FunctionTest, ShouldPass) {
  const double valid_angle_range = d2r(150.0);
  HeadingVoter voter;
  EXPECT_TRUE(voter.ShouldUseVoting());
  EXPECT_TRUE(voter.ShouldUseMeasurementHeading(0, 3.12, valid_angle_range));
  EXPECT_TRUE(voter.ShouldUseMeasurementHeading(0, 3.12, valid_angle_range));

  HeadingVoter voter1;
  voter1.Enable();
  EXPECT_TRUE(voter1.ShouldUseVoting());
  EXPECT_FALSE(voter1.ShouldUseMeasurementHeading(0, 0.01, valid_angle_range));
  EXPECT_FALSE(voter1.ShouldUseMeasurementHeading(0, 3.12, valid_angle_range));
  EXPECT_TRUE(voter1.ShouldUseMeasurementHeading(0, 3.12, valid_angle_range));
  EXPECT_TRUE(voter1.ShouldUseMeasurementHeading(0, 3.12, valid_angle_range));
  EXPECT_TRUE(voter1.ShouldUseMeasurementHeading(0, 3.12, valid_angle_range));

  voter1.Disnable();
  EXPECT_FALSE(voter1.ShouldUseVoting());
  voter1.Enable();
  EXPECT_TRUE(voter1.ShouldUseVoting());
  voter1.Disnable();
  EXPECT_FALSE(voter1.ShouldUseVoting());
  voter1.Reset();
  EXPECT_TRUE(voter1.ShouldUseVoting());
}

}  // namespace qcraft::tracker