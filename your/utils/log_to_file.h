#ifndef ONBOARD_UTILS_LOG_TO_FILE_H_
#define ONBOARD_UTILS_LOG_TO_FILE_H_

#include <fstream>
#include <string>
#include <vector>

namespace qcraft {
class LogToFile {
 public:
  explicit LogToFile(const std::string &filepath,
                     std::ios_base::openmode mode = std::ios_base::out,
                     bool instant_flush = false)
      : filepath_(filepath),
        instant_flush_(instant_flush),
        ofs_(filepath, mode) {}

  // for ascii output
  void AddLine(const std::string &line) {
    ofs_ << line << '\n';
    if (instant_flush_) ofs_ << std::flush;
  }

  // for binary output
  void Write(const uint8_t *begin, int count) {
    ofs_.write(reinterpret_cast<const char *>(begin), count);
    if (instant_flush_) ofs_ << std::flush;
  }

 private:
  std::string filepath_;
  bool instant_flush_;
  std::ofstream ofs_;
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_LOG_TO_FILE_H_
