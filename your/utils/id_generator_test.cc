#include "onboard/utils/id_generator.h"

#include "gtest/gtest.h"

namespace qcraft {

TEST(FilterUnitTest, Int32OK) {
  uint64_t last_uid = 0;
  for (int i = 0; i < 1000; i++) {
    const auto& cur_uid = GetUId();
    ASSERT_TRUE(cur_uid > last_uid);
    last_uid = cur_uid;
  }
}

}  // namespace qcraft
