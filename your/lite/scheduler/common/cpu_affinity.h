#ifndef ONBOARD_LITE_SCHEDULER_COMMON_CPU_AFFINITY_H_
#define ONBOARD_LITE_SCHEDULER_COMMON_CPU_AFFINITY_H_

#include <string>
#include <thread>
#include <vector>

namespace qcraft {

void ParseCpuset(const std::string& str, std::vector<int>* cpuset);

void SetProcessAffinity(const std::vector<int>& cpus);

void SetThreadAffinity(std::thread* thread, const std::vector<int>& cpus);

void SetSchedPolicy(std::thread* thread, std::string spolicy,
                    int sched_priority, pid_t tid = -1);

void DisableProcessAffinity();

}  // namespace qcraft

#endif  // ONBOARD_LITE_SCHEDULER_COMMON_CPU_AFFINITY_H_
