#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager_factory.h"

#include "onboard/lite/lite2_flags.h"
#include "onboard/lite/transport/shared_memory/lite_inner_process_shared_memory_manager.h"
#include "onboard/lite/transport/shared_memory/lite_shared_memory_manager.h"
#include "onboard/lite/transport/shared_memory/lite_shm_shared_memory_manager.h"

namespace qcraft {
SharedMemoryManager* ShmFactory::GetShmManager() {
  if (FLAGS_lite2_multiprocess) {
    static SharedMemoryManager* shm_manager = new ShmSharedMemoryManager();
    return shm_manager;
  } else {
    static SharedMemoryManager* shm_manager =
        new InnerProcessSharedMemoryManager();
    return shm_manager;
  }
}

};  // namespace qcraft
