#ifndef ONBOARD_GLOBAL_SYSTEM_IO_H_
#define ONBOARD_GLOBAL_SYSTEM_IO_H_

#include <cstdint>

namespace qcraft {

struct ProcessIoInfo {
  uint64_t rchar;        // characters read
  uint64_t wchar;        // characters written
  uint64_t syscr;        // number of read I/O operations
  uint64_t syscw;        // number of write I/O operations
  uint64_t read_bytes;   // number of bytes fetched from the storage layer
  uint64_t write_bytes;  // number of bytes sent to the storage layer
  uint64_t cancelled_write_bytes;  // number of bytes truncating pagecache
};

ProcessIoInfo GetSelfIoInfo();

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_SYSTEM_IO_H_
