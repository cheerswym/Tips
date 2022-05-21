// Copyright 2013 Google Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ONBOARD_UTILS_ERRORS_H_
#define ONBOARD_UTILS_ERRORS_H_

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "onboard/base/macros.h"
#include "onboard/base/port.h"

namespace qcraft {
namespace errors_internal {

// Extract the value or return a Status (with a different status_code and
// prefixed error message).  There's no value for a Status, so we just return
// the status.
inline absl::Status GetValueWithPrefix(absl::Status s,
                                       absl::StatusCode status_code,
                                       const std::string &error_msg_prefix) {
  if (LIKELY(s.ok())) return s;
  return absl::Status(status_code, error_msg_prefix + s.ToString());
}

// Extract the value or return a Status (with a different status_code and
// prefixed error message).
template <typename T, typename U>
inline absl::Status GetValueWithPrefix(absl::StatusOr<T> statusor, U *out_ptr,
                                       absl::StatusCode status_code,
                                       const std::string &error_msg_prefix) {
  if (LIKELY(statusor.ok())) {
    *out_ptr = *statusor;
    return statusor.status();
  }
  return absl::Status(status_code,
                      error_msg_prefix + statusor.status().ToString());
}

// Same as above but allows to return different Status (with different
// status_code and prefixed error message). D is a custom deleter (if any).
template <typename T, typename U, typename D>
inline absl::Status GetValueWithPrefix(absl::StatusOr<T *> statusor,
                                       ::std::unique_ptr<U, D> *out_ptr,
                                       absl::StatusCode status_code,
                                       const std::string &error_msg_prefix) {
  if (LIKELY(statusor.ok())) {
    out_ptr->reset(statusor.value());
    return statusor.status();
  }
  return absl::Status(status_code,
                      error_msg_prefix + statusor.status().ToString());
}

// Generically get a Status value from an argument expression (Status or
// StatusOr).  This could be made more public if we find utility in overloading
// it for other types.
inline absl::Status ToStatus(const absl::Status &status) { return status; }
template <typename T>
inline absl::Status ToStatus(const absl::StatusOr<T> &status_or_t) {
  return status_or_t.status();
}

}  // namespace errors_internal
}  // namespace qcraft

// checks that an expression (producing a Status or a StatusOr) was OK.  Use
// sparingly.  Prefer not to CHECK in production code.
#undef CHECK_OK  // defined by status.h, but only for Status, not StatusOr
#define CHECK_OK(expr)                                     \
  do {                                                     \
    const absl::Status &_status = (expr);                  \
    CHECK(qcraft::errors_internal::ToStatus(_status).ok()) \
        << _status.ToString();                             \
  } while (0)

// Macros defined below are working thanks to GCC extension.
#if defined(__GNUC__)

namespace qcraft {
namespace errors_internal {

inline const absl::Status XToStatus(const absl::Status &status) {
  return status;
}

template <typename T>
inline const absl::Status XToStatus(const absl::StatusOr<T> &statusor) {
  return statusor.status();
}

template <typename... Args>
inline const absl::Status XToStatus(const absl::Status &status,
                                    const Args &...rest) {
  return absl::Status(status.code(),
                      absl::StrCat(rest..., ": ", status.ToString()));
}

template <typename T, typename... Args>
inline const absl::Status XToStatus(const absl::StatusOr<T> &statusor,
                                    const Args &...rest) {
  return absl::Status(
      statusor.status().code(),
      absl::StrCat(rest..., ": ", statusor.status().ToString()));
}

template <typename... Args>
inline const absl::Status XToInternalStatus(const absl::Status &status) {
  return absl::InternalError(status.ToString());
}

template <typename T, typename... Args>
inline const absl::Status XToInternalStatus(const absl::StatusOr<T> &statusor) {
  return absl::InternalError(statusor.status().ToString());
}

template <typename... Args>
inline const absl::Status XToInternalStatus(const absl::Status &status,
                                            const Args &...rest) {
  return absl::InternalError(absl::StrCat(rest..., ": ", status.ToString()));
}

template <typename T, typename... Args>
inline const absl::Status XToInternalStatus(const absl::StatusOr<T> &statusor,
                                            const Args &...rest) {
  return absl::InternalError(
      absl::StrCat(rest..., ": ", statusor.status().ToString()));
}

inline void XToValue(absl::Status *status) {}

template <typename T>
inline T XToValue(absl::StatusOr<T> *statusor) {
  return statusor->value();
}

}  // namespace errors_internal
}  // namespace qcraft

#endif  // defined(__GNUC__)
#endif  // ONBOARD_UTILS_ERRORS_H_
