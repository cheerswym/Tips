#include "onboard/global/car_common.h"

#include <ostream>
#include <string>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(q_run_mode, "dsim",
              "The run mode of binary, must be one of onboard/dsim/rsim, See "
              "QRunMode for correspond type");
DEFINE_bool(on_test_bench, false, "Whether run on test bench");
DEFINE_bool(on_test_bench_for_rsim, false, "Whether run rsim on test bench");

namespace qcraft {

QRunMode QCraftRunMode() {
  if (FLAGS_q_run_mode == "onboard") {
    return Q_ONBOARD;
  }
  if (FLAGS_q_run_mode == "dsim") {
    return Q_DSIM;
  }
  if (FLAGS_q_run_mode == "rsim") {
    return Q_RSIM;
  }
  LOG(FATAL) << "Unknown q_run_mode: " << FLAGS_q_run_mode;
}

bool IsOnboardMode() { return FLAGS_q_run_mode == "onboard"; }

bool IsDSimMode() { return FLAGS_q_run_mode == "dsim"; }

bool IsRSimMode() { return FLAGS_q_run_mode == "rsim"; }

bool OnTestBench() { return FLAGS_on_test_bench; }

bool OnTestBenchForRsim() { return FLAGS_on_test_bench_for_rsim; }

}  // namespace qcraft
