#include "onboard/utils/stacktrace.h"

#include "absl/debugging/stacktrace.h"
#include "absl/debugging/symbolize.h"

namespace qcraft {
namespace {
static constexpr int kMaxStackDepth = 64;
void VisitTrace(void* stack_trace[], const int stack_depth,
                const std::function<void(int, const char*, void*)>& visitor) {
  for (int i = 0; i < stack_depth; ++i) {
    char out[1024];
    if (absl::Symbolize(stack_trace[i], out, sizeof(out))) {
      visitor(i, out, stack_trace[i]);
    } else {
      visitor(i, nullptr, stack_trace[i]);
    }
  }
}
}  // namespace

void GetStackTrace(std::ostream& os) {
  void* stack_trace[kMaxStackDepth];
  const int stack_depth = absl::GetStackTrace(stack_trace, kMaxStackDepth, 1);
  VisitTrace(stack_trace, stack_depth,
             [&](int index, const char* symbol, void* address) {
               if (symbol != nullptr) {
                 os << "#" << index << " " << symbol << " [" << address
                    << "]\n";
               } else {
                 os << "#" << index << " [" << address << "]\n";
               }
             });
}
}  // namespace qcraft
