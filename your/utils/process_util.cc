#include "onboard/utils/process_util.h"

#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"

namespace {
constexpr int kLockMode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
}

namespace qcraft {
namespace process {

int LockFile(int fd) {
  struct flock fl;
  fl.l_type = F_WRLCK;
  fl.l_start = 0;
  fl.l_whence = SEEK_SET;
  fl.l_len = 0;
  return fcntl(fd, F_SETLK, &fl);
}

absl::StatusOr<bool> AlreadyRunning(std::string_view program_name) {
  const auto lock_file = absl::StrFormat("/tmp/%s.pid", program_name);
  int fd = ::open(lock_file.c_str(), O_RDWR | O_CREAT, kLockMode);
  if (fd < 0) {
    return absl::UnknownError(
        absl::StrFormat("Can't open %s: %s", lock_file, strerror(errno)));
  }

  if (LockFile(fd) < 0) {
    if (errno == EACCES || errno == EAGAIN) {
      ::close(fd);
      return true;
    }
    return absl::UnknownError(
        absl::StrFormat("Can't lock %s: %s", lock_file, strerror(errno)));
  }

  (void)::ftruncate(fd, 0);
  const auto pid = std::to_string(::getpid());

  (void)::write(fd, pid.c_str(), pid.size());
  return false;
}

}  // namespace process
}  // namespace qcraft
