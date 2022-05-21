#include "onboard/utils/run_util.h"

#include "absl/strings/str_cat.h"
#include "boost/filesystem.hpp"
#include "offboard/vfs/vfs.h"
#include "onboard/global/constants.h"
#include "onboard/lite/check.h"

DEFINE_string(run, "", "Run name");
DEFINE_string(run_dir, "", "Directory for run");
DEFINE_double(start, 0.0, "Start time of run in seconds.");
DEFINE_double(end, 1e10, "End time of run in seconds.");

namespace qcraft::run_util {

std::string RunSegment::GetRunDir() const {
  return qcraft::run_util::GetRunDir(run);
}

std::string GetRunDir() {
  if (!FLAGS_run_dir.empty()) {
    return FLAGS_run_dir;
  }
  QCHECK_NE(FLAGS_run, "");
  return GetRunDir(FLAGS_run);
}

std::string GetRunDir(std::string run_name) {
  return VFS::GetRunPath(run_name);
}

std::string GetRunsTopDir() {
  return boost::filesystem::exists(
             absl::StrCat(kRunPrimaryPath, "/", FLAGS_run))
             ? kRunPrimaryPath
             : (boost::filesystem::exists(
                    absl::StrCat(kRunSecondaryPath, "/", FLAGS_run))
                    ? kRunSecondaryPath
                    : kRunThirdPath);
}

}  // namespace qcraft::run_util
