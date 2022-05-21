#include "onboard/global/system_mem.h"

#include <unistd.h>

#include <fstream>
#include <string>

namespace qcraft {
namespace {
constexpr char kSelfStatm[] = "/proc/self/statm";
constexpr char kProcMemInfo[] = "/proc/meminfo";
}  // namespace

GlobalMemInfo GetTotalMemUsage() {
  GlobalMemInfo info;
  std::ifstream file(kProcMemInfo, std::ios_base::in);
  std::string token;
  bool got_available = false;
  bool got_total = false;
  while (file >> token) {
    if (token == "MemAvailable:") {
      file >> info.available_mem_kb;
      got_available = true;

    } else if (token == "MemTotal:") {
      file >> info.total_mem_kb;
      got_total = true;
    }
    if (got_total && got_available) {
      break;
    }
  }
  file.close();
  return info;
}

uint64_t GetSelfMemUsage() {
  // 'file' statm seems to give the most reliable results
  std::ifstream stat_stream(kSelfStatm, std::ios_base::in);

  int64_t size, resident, share;

  stat_stream >> size >> resident >> share;  // don't care about the rest

  stat_stream.close();

  return (resident - share) * sysconf(_SC_PAGE_SIZE);
}

}  // namespace qcraft
