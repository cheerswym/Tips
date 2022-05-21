#include "onboard/utils/map_util.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(FindOrDie, Works) {
  std::map<int, int> m{{1, 1}, {2, 2}};
  EXPECT_EQ(FindOrDie(m, 2), 2);
}

}  // namespace
}  // namespace qcraft
