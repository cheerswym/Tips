#include "onboard/lite/transport/shm/shm_debug_util.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/lite/transport/shm/shm_manager.h"

namespace qcraft {
namespace shm {

TEST(ShmDebugTest, Allocate) {
  size_t hash = StringHash("test");
  EXPECT_EQ(
      qcraft::shm::DebugUtil::Instance()->TryFindChannelNameWithHash(hash),
      std::to_string(hash));

  qcraft::shm::DebugUtil::Instance()->hash_to_channel_name_.insert(
      std::make_pair(hash, "test"));
  EXPECT_EQ(
      qcraft::shm::DebugUtil::Instance()->TryFindChannelNameWithHash(hash),
      "test");
}

}  // namespace shm
}  // namespace qcraft
