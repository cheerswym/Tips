#ifndef ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_MANAGER_FACTORY_H_  // NOLINT
#define ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_MANAGER_FACTORY_H_  // NOLINT

#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager.h"

namespace qcraft {

class ShmFactory {
 public:
  static SharedMemoryManager* GetShmManager();
};

};  // namespace qcraft

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_LITE_TRANSPORT_SHARED_MEMORY_LITE_SHARED_MEMORY_MANAGER_FACTORY_H_
