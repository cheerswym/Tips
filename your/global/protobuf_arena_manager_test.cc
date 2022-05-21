#include "onboard/global/protobuf_arena_manager.h"

#include <thread>

#include "experimental/users/youliang/proto_buffer/proto/test.pb.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/proto/system_info.pb.h"

namespace qcraft {
namespace {
constexpr int number = 10000;
constexpr int test_proto_size = 64;  // Test4 proto size
}  // namespace
TEST(ArenaManagerTest, TestSingleThread) {
  FLAGS_arena_block_size = 1000000;
  auto* arena_manager = ArenaManager::GetArenaManager();
  EXPECT_TRUE(arena_manager->GetArena() != nullptr);
  EXPECT_EQ(arena_manager->GetArena()->SpaceAllocated(),
            FLAGS_arena_block_size);
  EXPECT_EQ(arena_manager->GetArena()->SpaceUsed(), 0);
  EXPECT_EQ(arena_manager->block_index, 0);
  EXPECT_EQ(arena_manager->arena_[0].get(), arena_manager->GetArena());

  for (int64_t i = 0; i < number; i++) {
    auto* test_proto = arena_manager->CreateArenaMessage<Test4>();
    EXPECT_TRUE(test_proto != nullptr);
    test_proto->add_d(12345);
  }
  EXPECT_EQ(arena_manager->GetArena()->SpaceUsed(), test_proto_size * number);
  EXPECT_EQ(arena_manager->block_index, 0);
  EXPECT_EQ(arena_manager->arena_[0].get(), arena_manager->GetArena());

  for (int64_t i = 0; i < number; i++) {
    auto* test_proto = arena_manager->CreateArenaMessage<Test4>();
    EXPECT_TRUE(test_proto != nullptr);
    test_proto->add_d(12345);
  }
  EXPECT_EQ(arena_manager->GetArena()->SpaceUsed(),
            test_proto_size * number * 2 - FLAGS_arena_block_size * 0.8);
  EXPECT_EQ(arena_manager->block_index, 1);
  EXPECT_EQ(arena_manager->arena_[1].get(), arena_manager->GetArena());
}

TEST(ArenaManagerTest, TestSingleThreadUniqueMessage) {
  FLAGS_arena_block_size = 1000000;
  auto* arena_manager = ArenaManager::GetArenaManager();
  EXPECT_TRUE(arena_manager->GetArena() != nullptr);
  EXPECT_EQ(arena_manager->GetArena()->SpaceAllocated(),
            FLAGS_arena_block_size);
  EXPECT_EQ(arena_manager->GetArena()->SpaceUsed(),
            test_proto_size * number * 2 - FLAGS_arena_block_size * 0.8);
  EXPECT_EQ(arena_manager->block_index, 1);
  EXPECT_EQ(arena_manager->arena_[1].get(), arena_manager->GetArena());

  std::unique_ptr<Test4, void (*)(Test4*)> test_proto =
      std::unique_ptr<Test4, void (*)(Test4*)>(
          arena_manager->CreateArenaMessage<Test4>(),
          [](Test4* proto) { EXPECT_TRUE(proto != nullptr); });

  EXPECT_TRUE(test_proto != nullptr);
  test_proto->add_d(12345);
}

}  // namespace qcraft
