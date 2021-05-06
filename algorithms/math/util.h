#ifndef ONBOARD_MATH_UTIL_H_
#define ONBOARD_MATH_UTIL_H_

#ifdef __SSE_4_1__
#ifdef __AVX512F__
#include <immintrin.h>
#else
#include <smmintrin.h>
#endif  // __AVX512F__
#endif  // __SSE_4_1__
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "onboard/lite/check.h"

namespace qcraft {

namespace math_util_internal {

#ifdef __SSE_4_1__

template <int kRoundingMode>
inline int RoundHelper(float val) {
#ifdef __AVX512F__
  return _mm_cvt_roundss_i32(_mm_load_ss(&val), kRoundingMode);
#else
  const auto t =
      _mm_round_ss(_mm_undefined_ps(), _mm_load_ss(&val), kRoundingMode);
  float rounded;
  _MM_EXTRACT_FLOAT(rounded, t, 0);
  return static_cast<int>(rounded);
#endif  // __AVX512F__
}

template <int kRoundingMode>
inline int RoundHelper(double val) {
#ifdef __AVX512F__
  return _mm_cvt_roundsd_i32(_mm_load_ss(&val), kRoundingMode);
#else
  const auto rounded = _mm_cvtsd_f64(
      _mm_round_sd(_mm_undefined_pd(), _mm_load_sd(&val), kRoundingMode));
  return static_cast<int>(rounded);
#endif  // __AVX512F__
}

#endif  // __SSE_4_1__

}  // namespace math_util_internal

template <typename T>
inline int FloorToInt(T val) {
#ifdef __SSE_4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_NEG_INF |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(std::floor(val));
#endif
}

template <typename T>
inline int CeilToInt(T val) {
#ifdef __SSE_4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_POS_INF |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(std::ceil(val));
#endif
}

// Note that RoundToInt rounds half to even.
// E.g., it rounds 1.5 to 2, and 2.5 to 2.
template <typename T>
inline int RoundToInt(T val) {
#ifdef __SSE_4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_NEAREST_INT |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(std::round(val));
#endif
}

template <typename T>
inline int TruncateToInt(T val) {
#ifdef __SSE_4_1__
  return math_util_internal::RoundHelper<_MM_FROUND_TO_ZERO |
                                         _MM_FROUND_NO_EXC>(val);
#else
  return static_cast<int>(val);
#endif
}

template <typename T>
inline T Sign(T val) {
  if (val > 0) return 1;
  if (val < 0) return -1;
  return 0;
}

template <typename T>
inline bool IsInt(T val) {
  return val == TruncateToInt(val);
}

template <typename T>
bool InRange(T val, T bound0, T bound1) {
  if (bound0 > bound1) std::swap(bound0, bound1);
  return val >= bound0 && val <= bound1;
}

template <typename T>
constexpr T d2r(T d) {
  static_assert(std::is_floating_point<T>::value);
  return d * T(M_PI / 180.);
}
template <typename T>
constexpr T r2d(T r) {
  static_assert(std::is_floating_point<T>::value);
  return r * T(180. / M_PI);
}

inline constexpr double Mph2Mps(double mph) { return mph * 1609.34 / 3600; }
inline constexpr double Mps2Mph(double mps) { return mps / 1609.34 * 3600; }

inline constexpr double Kph2Mps(double kph) { return kph * 1000.0 / 3600; }
inline constexpr double Mps2Kph(double mps) { return mps / 1000.0 * 3600; }

template <typename T>
constexpr T Sqr(T val) {
  return val * val;
}

template <typename T>
constexpr T Cube(T val) {
  return val * val * val;
}

template <typename T>
constexpr T Quar(T val) {
  return val * val * val * val;
}

template <typename T, typename U>
constexpr std::enable_if_t<std::is_integral_v<U>, T> Power(T val, U exp) {
  QCHECK(exp >= 0);
  T res = 1;
  while (exp > 0) {
    if (exp & 1) res *= val;
    exp >>= 1;
    val *= val;
  }
  return res;
}

template <typename T>
T Hypot(T a, T b) {
  return std::sqrt(Sqr(a) + Sqr(b));
}

template <typename T>
T Hypot(T a, T b, T c) {
  return std::sqrt(Sqr(a) + Sqr(b) + Sqr(c));
}

template <typename T, typename TS>
constexpr T Lerp(T a, T b, TS s) {
  return a + (b - a) * s;
}

template <typename T>
constexpr T LerpFactor(T a, T b, T value) {
  if (b == a) return 0.0;
  return (value - a) / (b - a);
}

template <typename T>
constexpr T WrapIfNegative(T val, T range) {
  return val < 0.0 ? val + range : val;
}

// Wrap val to range [min_val, max_val).
template <typename T>
constexpr T WrapToRange(T val, T min_val, T max_val) {
  return WrapIfNegative(std::fmod(val - min_val, max_val - min_val),
                        max_val - min_val) +
         min_val;
}

// Normalize an angle in radius into [-pi, pi).
template <typename T>
T NormalizeAngle(T angle) {
  // Fast path.
  if (LIKELY(std::abs(angle) < T(M_PI))) return angle;
  if (std::abs(angle) > T(M_PI * 10)) {
    angle = std::fmod(angle, 2 * M_PI);
  }
  if (angle < T(-M_PI)) {
    do {
      angle += T(M_PI * 2.0);
    } while (angle < T(-M_PI));
  } else {
    while (angle >= T(M_PI)) angle -= T(M_PI * 2.0);
  }
  return angle;
}

// Wrap angle to [0, 2 * PI).
template <typename T>
T WrapAngle(T angle) {
  T new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < T(0.0) ? new_angle + T(M_PI * 2.0) : new_angle;
}

template <typename T, typename TS>
constexpr T LerpAngle(T a, T b, TS s) {
  return a + NormalizeAngle(b - a) * s;
}

// Calculate the difference between angle from and to. The returned range is
// between [-PI, PI).
template <typename T>
constexpr T AngleDifference(T from, T to) {
  return NormalizeAngle(to - from);
}

// Compute the average angle of a group of angles in radians.
inline double MeanAngle(const std::vector<double>& angles) {
  if (angles.empty()) return 0.0;
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const auto angle : angles) {
    sin_sum += std::sin(angle);
    cos_sum += std::cos(angle);
  }
  return std::atan2(sin_sum, cos_sum);
}

// Get the opposite direction's angle.
inline double OppositeAngle(double angle) {
  return NormalizeAngle(angle + M_PI);
}

template <typename T>
inline T SafeClamp(const T x, const T min, const T max,
                   bool failure_upon_exception = true) {
  constexpr double kEpsilon = 1e-8;
  if (min > max - kEpsilon) {
    VLOG(0) << absl::StrFormat(
        "Illegal clamp config: min = %7.6f > max = %7.6f !", min, max);
  }
  if (failure_upon_exception) CHECK_LE(min, max);
  return std::clamp(x, min, max);
}

template <typename T>
inline void UpdateMax(const T x, T* max_val) {
  *max_val = std::max(x, *max_val);
}

template <typename T>
inline void UpdateMin(const T x, T* min_val) {
  *min_val = std::min(x, *min_val);
}

// Returns the real root of ax^2 + bx + c = 0.0. The roots are in non-decreasing
// orders and complex roots are ignored.
std::vector<double> QuadraticRoot(double a, double b, double c);

}  // namespace qcraft

#endif  // ONBOARD_MATH_UTIL_H_
