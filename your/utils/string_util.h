#ifndef ONBOARD_UTILS_STRING_UTIL_H_
#define ONBOARD_UTILS_STRING_UTIL_H_

#include <string>
#include <vector>

#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"

namespace qcraft {

// Transform a vector of vec2d to string.
template <typename T>
inline std::string VecOfVec2dToString(const std::vector<T> data) {
  return absl::StrJoin(data, ", ", [](std::string *out, const T x) {
    absl::StrAppendFormat(out, "(%5.4f, %5.4f)", x[0], x[1]);
  });
}

// Transform a vector of rational to string.
template <typename T>
inline std::string VecOfRealNumbersToString(const std::vector<T> data) {
  return absl::StrJoin(data, ", ", [](std::string *out, const T x) {
    absl::StrAppendFormat(out, "%5.4f", x);
  });
}

template <typename Iter>
inline std::string VecOfIntsToString(Iter begin, Iter end) {
  return absl::StrJoin(begin, end, ", ", [](std::string *out, const int x) {
    absl::StrAppendFormat(out, "%5d", x);
  });
}

}  // namespace qcraft

#endif  // ONBOARD_UTILS_STRING_UTIL_H_
