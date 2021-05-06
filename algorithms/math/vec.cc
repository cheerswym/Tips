#include "onboard/math/vec.h"

namespace qcraft {

std::string DebugStringFullPrecision(absl::Span<const Vec2d> vec) {
  const auto to_str = [](std::string *out, const Vec2d v) {
    return out->append(v.DebugStringFullPrecision());
  };
  return absl::StrJoin(vec, ", ", to_str);
}

}  // namespace qcraft
