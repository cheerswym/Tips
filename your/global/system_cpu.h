#ifndef ONBOARD_GLOBAL_SYSTEM_CPU_H_
#define ONBOARD_GLOBAL_SYSTEM_CPU_H_

#include <cstdint>
#include <vector>

namespace qcraft {

// return current total cpu usage 1 core is 100, up to total core * 100
void GetTotalCPU(std::vector<uint64_t>* current_total_cpu);

// return current process's cpu usage 1 core is 100.
uint64_t GetSelfCPU();

std::vector<uint32_t> GetCPUFreq();

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_SYSTEM_CPU_H_
