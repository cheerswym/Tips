#ifndef ONBOARD_UTILS_RUN_UTIL_H_
#define ONBOARD_UTILS_RUN_UTIL_H_

#include <string>

#include "gflags/gflags.h"

DECLARE_string(run);
DECLARE_string(run_dir);
DECLARE_double(start);
DECLARE_double(end);

namespace qcraft::run_util {

struct RunSegment {
  RunSegment() : RunSegment(FLAGS_run, FLAGS_start, FLAGS_end) {}
  RunSegment(const std::string& run, double start, double end)
      : run(run), start(start), end(end) {}

  std::string GetRunDir() const;

  // A run can be a standard run name, or a directory containing the run.
  std::string run;
  double start = 0.0;
  double end = 0.0;
};

// Get the directory of the currently run.
std::string GetRunDir();

// Get the directory of run.
std::string GetRunDir(std::string run_name);

// Get the top directory
std::string GetRunsTopDir();

}  // namespace qcraft::run_util

#endif  // ONBOARD_UTILS_RUN_UTIL_H_
