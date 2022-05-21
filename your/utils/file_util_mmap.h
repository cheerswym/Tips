#ifndef ONBOARD_UTILS_FILE_UTIL_MMAP_H_
#define ONBOARD_UTILS_FILE_UTIL_MMAP_H_

#include <sys/mman.h>

#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "onboard/global/singleton.h"

namespace qcraft::file_util {
class MMapFile {
 public:
  explicit MMapFile(const std::string& filename, bool writable = false);
  ~MMapFile() { Close(); }
  bool GetFileContent(std::string* content) const;
  std::string GetFileContentOrDie() const;
  bool GetFileContentView(std::string_view* content) const;
  std::string_view GetFileContentViewOrDie() const;
  void Close();
  absl::Status status() const { return status_; }
  size_t size() const { return file_size_; }
  void* data() { return mmap_ptr_; }

 private:
  void* mmap_ptr_ = MAP_FAILED;
  char* char_ptr_ = nullptr;
  absl::Status status_;
  size_t file_size_ = 0;
  DISALLOW_COPY_AND_ASSIGN(MMapFile);
};
}  // namespace qcraft::file_util

#endif  //  ONBOARD_UTILS_FILE_UTIL_MMAP_H_
