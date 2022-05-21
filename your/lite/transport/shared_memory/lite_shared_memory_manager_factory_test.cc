#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager_factory.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace shm {

TEST(SharedMemoryManagerTest, MultiProcess) {
  FLAGS_lite2_multiprocess = true;
  EXPECT_NE(ShmFactory::GetShmManager(), nullptr);
}

TEST(SharedMemoryManagerTest, InnerProcess) {
  FLAGS_lite2_multiprocess = false;
  EXPECT_NE(ShmFactory::GetShmManager(), nullptr);
}

}  // namespace shm
}  // namespace qcraft
