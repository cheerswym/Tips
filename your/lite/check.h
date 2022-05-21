#ifndef ONBOARD_LITE_CHECK_H_
#define ONBOARD_LITE_CHECK_H_

#include <string>
#include <utility>

#include "onboard/lite/check_helper.h"

#define Q_ONBOARD_BUILD  // turn it on
#ifdef Q_ONBOARD_BUILD

#define QCHECK(condition) QCHECK_WITH_MODULE(condition)
#define QCHECK_NOTNULL(val) \
  CheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))
#define QCHECK_EQ(val1, val2) QCHECK_EQ_WITH_MODULE(val1, val2)
#define QCHECK_NE(val1, val2) QCHECK_NE_WITH_MODULE(val1, val2)
#define QCHECK_LE(val1, val2) QCHECK_LE_WITH_MODULE(val1, val2)
#define QCHECK_LT(val1, val2) QCHECK_LT_WITH_MODULE(val1, val2)
#define QCHECK_GE(val1, val2) QCHECK_GE_WITH_MODULE(val1, val2)
#define QCHECK_GT(val1, val2) QCHECK_GT_WITH_MODULE(val1, val2)
#define QCHECK_NEAR(val1, val2, margin) \
  QCHECK_NEAR_WITH_MODULE(val1, val2, margin)
#define QCHECK_OK(expr)                                           \
  do {                                                            \
    const absl::Status &_status = (expr);                         \
    QCHECK_WITH_MODULE(_status.ok())                              \
        << "'" #expr "' returned status: " << _status.ToString(); \
  } while (0)

#else

#define QCHECK(condition) CHECK(condition)
#define QCHECK_NOTNULL(ptr) CHECK_NOTNULL(ptr)
#define QCHECK_EQ(val1, val2) CHECK_EQ(val1, val2)
#define QCHECK_NE(val1, val2) CHECK_NE(val1, val2)
#define QCHECK_LE(val1, val2) CHECK_LE(val1, val2)
#define QCHECK_LT(val1, val2) CHECK_LT(val1, val2)
#define QCHECK_GE(val1, val2) CHECK_GE(val1, val2)
#define QCHECK_GT(val1, val2) CHECK_GT(val1, val2)
#define QCHECK_NEAR(val1, val2, margin) CHECK_NEAR(val1, val2, margin)
#define QCHECK_OK(condition) CHECK(condition.ok())

#endif  // Q_ONBOARD_BUILD

#endif  // ONBOARD_LITE_CHECK_H_
