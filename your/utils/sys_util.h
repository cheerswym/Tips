#pragma once

#include <string>
#include <string_view>

#include "absl/status/statusor.h"

namespace qcraft {

// @desc  Get hostname by parsing hostname file, e.g. "/etc/hostname"
absl::StatusOr<std::string> GetHostNameByFile(
    std::string_view fpath = "/etc/hostname");
absl::StatusOr<std::string> GetHostNameByAPI();

}  // namespace qcraft
