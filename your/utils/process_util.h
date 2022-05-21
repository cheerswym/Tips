#pragma once

#include <string_view>

#include "absl/status/statusor.h"

namespace qcraft {
namespace process {

int LockFile(int fd);

/**
 * @desc: Whether the program named program_name is already running
 */
absl::StatusOr<bool> AlreadyRunning(std::string_view program_name);

}  // namespace process
}  // namespace qcraft
