#ifndef ONBOARD_UTILS_PROGRESS_BAR_H_
#define ONBOARD_UTILS_PROGRESS_BAR_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "onboard/lite/check.h"

namespace qcraft {
typedef uint64_t uint64;
class ProgressBar {
  // reference: https://github.com/htailor/cpp_progress_bar
 public:
  ProgressBar() = default;
  ProgressBar(const uint64 n, const std::string& discription = "",
              std::ostream& out = std::clog);
  ProgressBar(const ProgressBar& progress_bar);
  void SetFrequencyUpdate(uint64 frequency_update);
  void SetUpdateGap(uint64 gap);
  void SetStyle(const std::string& unit_bar, const std::string& unit_space);
  void Progressed(uint64 idx);

 private:
  uint64 n_;
  uint64 desc_width_;
  uint64 frequency_update_;
  std::mutex mutex_os_;
  std::ostream* os_pointer_;  // a nake pointer is used here because the object
                              // will not be created by "new", it only get the
                              // address of std::cout or std::cerr

  std::string description_;
  std::string unit_bar_;
  std::string unit_space_;

  void ClearBarField();
  int GetConsoleWidth();
  int GetBarLength();
};
}  // namespace qcraft

#endif  // ONBOARD_UTILS_PROGRESS_BAR_H_
