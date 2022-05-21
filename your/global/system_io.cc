#include "onboard/global/system_io.h"

#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

namespace qcraft {
namespace {
constexpr char kSelfIo[] = "/proc/self/io";
}  // namespace

ProcessIoInfo GetSelfIoInfo() {
  // 'file' statm seems to give the most reliable results
  std::ifstream io_stream(kSelfIo, std::ios_base::in);
  ProcessIoInfo io_info;

  std::string str;
  uint64_t* value = &io_info.rchar;
  for (int i = 0; i < 7; i++) {
    getline(io_stream, str);
    *value++ = std::stol(str.substr(str.find_last_of(':') + 1));
  }
  io_stream.close();

  return io_info;
}

}  // namespace qcraft
