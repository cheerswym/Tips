#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"

#include "gtest/gtest.h"

namespace qcraft::tracker {

TEST(TrackerBasicDataStructrueTest, AllDataStructure) {
  using State = Vector<double, 6>;
  using P = Covariance<State>;
  EXPECT_EQ(P::RowsAtCompileTime, P::ColsAtCompileTime);
  EXPECT_EQ(State::RowsAtCompileTime, P::ColsAtCompileTime);
  EXPECT_EQ(State::RowsAtCompileTime, 6);

  State s;
  s << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  P F;
  F.setIdentity();
  State x1 = F * s;

  EXPECT_DOUBLE_EQ(x1(0), 1.0);
  EXPECT_DOUBLE_EQ(x1(1), 1.0);
  EXPECT_DOUBLE_EQ(x1(2), 1.0);
  EXPECT_DOUBLE_EQ(x1(3), 1.0);
  EXPECT_DOUBLE_EQ(x1(4), 1.0);
  EXPECT_DOUBLE_EQ(x1(5), 1.0);
}
}  // namespace qcraft::tracker
