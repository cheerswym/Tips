#include "onboard/utils/sys_util.h"

#include <sys/utsname.h>

#include <cerrno>
#include <cstring>
#include <fstream>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"

namespace qcraft {

absl::StatusOr<std::string> GetHostNameByFile(std::string_view fpath) {
  std::ifstream fin(fpath.data());
  if (!fin) {
    return absl::InternalError(absl::StrCat("Failed to open ", fpath));
  }
  std::string res;
  std::getline(fin, res);
  fin.close();
  return res;
}

absl::StatusOr<std::string> GetHostNameByAPI() {
  struct utsname buf;
  std::memset(&buf, 0, sizeof(buf));

  int ret = uname(&buf);
  if (ret != 0) {
    return absl::InternalError(absl::StrCat(errno, ":", strerror(errno)));
  }
  return std::string(buf.nodename);
}

}  // namespace qcraft
