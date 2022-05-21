#include "onboard/lite/transport/shm/shm_flags.h"

DEFINE_int64(transport_managed_shared_memory_size, 6LL * 1024 * 1024 * 1024,
             "Max size of managed shared memory size in bytes");
DEFINE_string(transport_managed_shared_memory_name, "qcraft",
              "Name of managed shared memory");
