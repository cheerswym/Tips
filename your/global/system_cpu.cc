#include "onboard/global/system_cpu.h"

#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

namespace qcraft {
namespace {
constexpr char kSelfStat[] = "/proc/self/stat";
constexpr char KStat[] = "/proc/stat";
constexpr char KInfo[] = "/proc/cpuinfo";
}  // namespace

void GetTotalCPU(std::vector<uint64_t>* current_total_cpu) {
  std::ifstream stream(KStat, std::ios_base::in);
  std::string name, user, nice, system, invalid;
  std::vector<std::string> str(11);
  for (int i = 0; i < current_total_cpu->size(); i++) {
    stream >> name >> user >> nice >> system >> invalid >> invalid >> invalid >>
        invalid >> invalid >> invalid >> invalid;
    (*current_total_cpu)[i] = stoull(user) + stoull(nice) + stoull(system);
  }
  stream.close();
}

std::vector<uint32_t> GetCPUFreq() {
  std::vector<uint32_t> res;
  std::ifstream stream(KInfo, std::ios_base::in);
  std::string line;
  std::smatch match;
  std::regex mhz_info_re("cpu MHz\\s+(\\d+)");
  while (std::getline(stream, line)) {
    if (std::regex_search(line, match, mhz_info_re) && match.size() >= 2) {
      res.emplace_back(stoul(match[1].str()));
    }
  }
  return res;
}

uint64_t GetSelfCPU() {
  std::ifstream stat_stream(kSelfStat, std::ios_base::in);

  // Dummy vars for leading entries in stat that we don't care about
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime;

  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr >>
      tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt >> utime >>
      stime;

  stat_stream.close();
  return stoull(utime) + stoull(stime);
}
}  // namespace qcraft
