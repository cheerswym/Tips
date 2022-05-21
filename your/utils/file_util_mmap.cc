#include "onboard/utils/file_util_mmap.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "absl/strings/str_format.h"
#include "onboard/lite/check.h"

namespace qcraft::file_util {
MMapFile::MMapFile(const std::string& filename, bool writable) {
  int fd = open(filename.c_str(), writable ? O_RDWR : O_RDONLY);
  if (fd < 0) {
    status_ = absl::PermissionDeniedError(
        absl::StrFormat("open %s: %s", filename, strerror(errno)));
    return;
  }
  struct stat statbuf;
  if (fstat(fd, &statbuf) < 0) {
    status_ = absl::InternalError(
        absl::StrFormat("stat %s: %s", filename, strerror(errno)));
    ::close(fd);
    return;
  }
  file_size_ = statbuf.st_size;
  const int mode = (writable ? (PROT_READ | PROT_WRITE) : PROT_READ);
  mmap_ptr_ = mmap(nullptr, statbuf.st_size, mode, MAP_SHARED, fd, 0);
  char_ptr_ = static_cast<char*>(mmap_ptr_);
  if (mmap_ptr_ == MAP_FAILED) {
    status_ = absl::InternalError(
        absl::StrFormat("mmap %s: %s", filename, strerror(errno)));
  } else {
    status_ = absl::OkStatus();
  }
  ::close(fd);
}

std::string_view MMapFile::GetFileContentViewOrDie() const {
  QCHECK_OK(status_);
  return std::string_view(char_ptr_, file_size_);
}

std::string MMapFile::GetFileContentOrDie() const {
  QCHECK_OK(status_);
  return std::string(char_ptr_, char_ptr_ + file_size_);
}

bool MMapFile::GetFileContentView(std::string_view* content) const {
  if (!status_.ok()) return false;
  *QCHECK_NOTNULL(content) = std::string_view(char_ptr_, file_size_);
  return true;
}

bool MMapFile::GetFileContent(std::string* content) const {
  if (!status_.ok()) return false;
  *QCHECK_NOTNULL(content) = std::string(char_ptr_, char_ptr_ + file_size_);
  return true;
}

void MMapFile::Close() {
  if (mmap_ptr_ != MAP_FAILED) {
    munmap(mmap_ptr_, file_size_);
    status_ = absl::UnavailableError("MMap is closed");
    mmap_ptr_ = MAP_FAILED;
    file_size_ = 0;
  }
}
}  // namespace qcraft::file_util
