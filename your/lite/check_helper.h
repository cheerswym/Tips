#ifndef ONBOARD_LITE_CHECK_HELPER_H_
#define ONBOARD_LITE_CHECK_HELPER_H_

#include <utility>

#include "onboard/global/buffered_logger.h"
#include "onboard/global/logging.h"
#include "onboard/lite/lite2_flags.h"
#include "onboard/proto/buffered_log.pb.h"

template <typename T>
T CheckNotNull(const char* file, int line, const char* msg, T&& t) {
  if (t == nullptr) {
    qcraft::BufferedLoggerWrapper(file, line, qcraft::LogSeverity::FATAL)
            .stream()
        << msg;
  }
  return std::forward<T>(t);
}
#define QCHECK_WITH_MODULE(condition) \
  QLOG_WITH_MODULE_IF(FATAL, !(condition)) << "Check failed: " #condition " "

#define QCHECK_OP_WITH_MODULE(op, val1, val2) \
  QCHECK_WITH_MODULE((val1)op(val2))          \
      << "(" << (val1) << " " #op " " << (val2) << ") "

#define QCHECK_EQ_WITH_MODULE(val1, val2) QCHECK_OP_WITH_MODULE(==, val1, val2)

#define QCHECK_NE_WITH_MODULE(val1, val2) QCHECK_OP_WITH_MODULE(!=, val1, val2)

#define QCHECK_LE_WITH_MODULE(val1, val2) QCHECK_OP_WITH_MODULE(<=, val1, val2)

#define QCHECK_LT_WITH_MODULE(val1, val2) QCHECK_OP_WITH_MODULE(<, val1, val2)

#define QCHECK_GE_WITH_MODULE(val1, val2) QCHECK_OP_WITH_MODULE(>=, val1, val2)

#define QCHECK_GT_WITH_MODULE(val1, val2) QCHECK_OP_WITH_MODULE(>, val1, val2)

// Reference implementation
// https://github.com/google/glog/blob/503e3dec8d1fe071376befc62119a837c26612a3/src/glog/logging.h.in#L952
#define QCHECK_NEAR_WITH_MODULE(val1, val2, margin)   \
  do {                                                \
    QCHECK_GE_WITH_MODULE((val1), (val2) - (margin)); \
    QCHECK_LE_WITH_MODULE((val1), (val2) + (margin)); \
  } while (0)

#endif  // ONBOARD_LITE_CHECK_HELPER_H_
