#pragma once
#include <google/protobuf/arena.h>

#include <memory>
#include <mutex>
#include <vector>

#include "glog/logging.h"
#include "onboard/global/counter.h"
#include "onboard/global/protobuf_flags.h"
namespace qcraft {
class ArenaManager {
 public:
  static ArenaManager* GetArenaManager() {
    static ArenaManager* arena_manager = new ArenaManager();
    static std::once_flag flag;
    std::call_once(flag, [&] {
      arena_manager->arena_block_[0].resize(FLAGS_arena_block_size);
      google::protobuf::ArenaOptions options;
      options.initial_block = &arena_manager->arena_block_[0][0];
      options.initial_block_size = arena_manager->arena_block_[0].size();
      arena_manager->arena_[0] =
          std::make_unique<google::protobuf::Arena>(options);

      google::protobuf::ArenaOptions option1;
      arena_manager->arena_block_[1].resize(FLAGS_arena_block_size);
      option1.initial_block = &arena_manager->arena_block_[1][0];
      option1.initial_block_size = arena_manager->arena_block_[1].size();
      arena_manager->arena_[1] =
          std::make_unique<google::protobuf::Arena>(option1);
    });

    return arena_manager;
  }
  template <typename T>
  T* CreateArenaMessage() {
    return google::protobuf::Arena::CreateMessage<T>(GetArena());
  }

 private:
  ArenaManager() {}
  google::protobuf::Arena* GetArena() {
    if (arena_[block_index]->SpaceUsed() >= FLAGS_arena_block_size * 0.8) {
      QCOUNTER("ProtoArenaSwitch", 1);
      block_index ^= 1;
      arena_[block_index]->Reset();
    }
    return arena_[block_index].get();
  }

 private:
  int block_index = 0;
  std::array<std::vector<char>, 2> arena_block_;
  std::array<std::unique_ptr<google::protobuf::Arena>, 2> arena_;
};
}  // namespace qcraft
