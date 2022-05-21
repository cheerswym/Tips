#include "onboard/lite/scheduler/common/cpu_affinity.h"

#include <sched.h>
#include <sys/resource.h>

#include "glog/logging.h"
#include "onboard/lite/check.h"

namespace qcraft {

void ParseCpuset(const std::string& str, std::vector<int>* cpuset) {
  std::vector<std::string> lines;
  std::stringstream ss(str);
  std::string l;
  while (getline(ss, l, ',')) {
    lines.push_back(l);
  }
  for (auto line : lines) {
    std::stringstream ss(line);
    std::vector<std::string> range;
    while (getline(ss, l, '-')) {
      range.push_back(l);
    }
    if (range.size() == 1) {
      cpuset->push_back(std::stoi(range[0]));
    } else if (range.size() == 2) {
      for (int i = std::stoi(range[0]), e = std::stoi(range[1]); i <= e; i++) {
        cpuset->push_back(i);
      }
    } else {
      LOG(FATAL) << "Parsing cpuset format error.";
    }
  }
}

void SetProcessAffinity(const std::vector<int>& cpus) {
  cpu_set_t set;
  CPU_ZERO(&set);

  for (const auto cpu : cpus) {
    CPU_SET(cpu, &set);
  }
  if (sched_setaffinity(getpid(), sizeof(set), &set) == -1) {
    LOG(ERROR) << "ERROR: Could not set CPU Affinity, continuing...";
  }
}

void DisableProcessAffinity() {
  cpu_set_t set;

  for (int i = 0; i < _SC_NPROCESSORS_CONF; i++) {
    CPU_SET(i, &set);
  }
  if (sched_setaffinity(getpid(), sizeof(set), &set) == -1) {
    LOG(ERROR) << "ERROR: Could not set CPU Affinity, continuing...";
  }
}

void SetThreadAffinity(std::thread* thread, const std::vector<int>& cpus) {
  cpu_set_t set;
  CPU_ZERO(&set);

  for (const auto cpu : cpus) {
    CPU_SET(cpu, &set);
  }
  pthread_setaffinity_np(thread->native_handle(), sizeof(set), &set);
}

void SetSchedPolicy(std::thread* thread, std::string spolicy,
                    int sched_priority, pid_t tid) {
  struct sched_param sp;
  int policy;

  memset(reinterpret_cast<void*>(&sp), 0, sizeof(sp));
  sp.sched_priority = sched_priority;

  if (!spolicy.compare("SCHED_FIFO")) {
    policy = SCHED_FIFO;
    pthread_setschedparam(thread->native_handle(), policy, &sp);
  } else if (!spolicy.compare("SCHED_RR")) {
    policy = SCHED_RR;
    pthread_setschedparam(thread->native_handle(), policy, &sp);
  } else if (!spolicy.compare("SCHED_OTHER")) {
    setpriority(PRIO_PROCESS, tid, sched_priority);
  }
}

}  // namespace qcraft
