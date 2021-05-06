#ifndef ONBOARD_MATH_FAST_MATH_H_
#define ONBOARD_MATH_FAST_MATH_H_

#include <algorithm>
#include <cmath>
#include <tuple>

#include "glog/logging.h"
#include "onboard/math/sin_table.h"
#include "onboard/math/util.h"

namespace qcraft::fast_math {

// Fast implementation of atan2. Maximum error is approx 0.012 degree.
// max(abs(x), abx(y)) must be non-zero.
template <typename T>
inline T Atan2(T y, T x) {
  const T abs_x = std::abs(x);
  const T abs_y = std::abs(y);
  T min_val, max_val;
  const bool x_lt_y = abs_x < abs_y;
  if (x_lt_y) {
    min_val = abs_x;
    max_val = abs_y;
  } else {
    min_val = abs_y;
    max_val = abs_x;
  }
  if (max_val == T(0.0)) return 0.0;
  const T a = min_val / max_val;
  const T s = a * a;
  T r =
      ((T(-0.0464964749) * s + T(0.15931422)) * s - T(0.327622764)) * s * a + a;
  if (x_lt_y) r = T(M_PI_2) - r;
  if (x < 0) r = T(M_PI) - r;
  if (y < 0) r = -r;
  return r;
}

// Maximum error is ~5e-5. The input angle must be normalized.
template <typename T>
inline T SinNormalized(T angle) {
  constexpr T kScale = M_1_PI * (kSinTableSize - 1) * 2.0;
  if (angle < 0.0f) {
    if (angle < -M_PI_2) {
      const int index = RoundToInt((angle + T(M_PI)) * kScale);
      return -kSinTable[index];
    }
    const int index = RoundToInt(-angle * kScale);
    return -kSinTable[index];
  }
  if (angle < M_PI_2) {
    const int index = RoundToInt(angle * kScale);
    return kSinTable[index];
  }
  const int index = RoundToInt((T(M_PI) - angle) * kScale);
  return kSinTable[index];
}

// Maximum error is ~5e-5. The input angle must be normalized.
template <typename T>
inline T CosNormalized(T angle) {
  constexpr T kScale = M_1_PI * (kSinTableSize - 1) * 2.0;
  if (angle < T(0.0)) {
    if (angle < T(-M_PI_2)) {
      const int index = RoundToInt((-angle - T(M_PI_2)) * kScale);
      return -kSinTable[index];
    }
    const int index = RoundToInt((angle + T(M_PI_2)) * kScale);
    return kSinTable[index];
  }
  if (angle < T(M_PI_2)) {
    const int index = RoundToInt((T(M_PI_2) - angle) * kScale);
    return kSinTable[index];
  }
  const int index = RoundToInt((angle - T(M_PI_2)) * kScale);
  return -kSinTable[index];
}

template <typename T>
inline T Sin(T angle) {
  return SinNormalized(NormalizeAngle(angle));
}

template <typename T>
inline T Cos(T angle) {
  return CosNormalized(NormalizeAngle(angle));
}

}  // namespace qcraft::fast_math

#endif  // ONBOARD_MATH_FAST_MATH_H_
