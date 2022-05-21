#ifndef ONBOARD_GLOBAL_SYSTEM_MEM_H_
#define ONBOARD_GLOBAL_SYSTEM_MEM_H_

#include <cstdint>

namespace qcraft {

struct GlobalMemInfo {
  uint64_t available_mem_kb;
  uint64_t total_mem_kb;
};

// Attempts to read the system-dependent data for a process' virtual memory
// size in byte
uint64_t GetSelfMemUsage();

GlobalMemInfo GetTotalMemUsage();

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_SYSTEM_MEM_H_
