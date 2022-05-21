#ifndef ONBOARD_GLOBAL_TRACE_UTIL_H_
#define ONBOARD_GLOBAL_TRACE_UTIL_H_

#include <string>

#include "onboard/proto/trace.pb.h"

namespace qcraft {

struct TraceEvent {
  const char* name;
  struct TraceArg {
    const char* key;
    static constexpr int kValueMaxLength = 63;
    char value[kValueMaxLength + 1];
  };
  int num_args = 0;
  TraceArg args[3];
  double start = 0.0;
  double end = 0.0;
  bool use_ftrace = true;
};

namespace trace_util {

const char kCompressTag = 'x';

template <typename T>
std::string ToString(T t) {
  return std::to_string(t);
}

inline std::string ToString(const std::string& str) { return str; }

// Convert a trace proto into JSON string that is read by chrome tracing tool.
std::string TraceProtoToJson(const TraceProto& trace_proto);

std::string TraceProtoToKernelEvent(const TraceProto& trace_proto);

bool Compress(const std::string& src, std::string* dest);

bool Decompress(const std::string& src, std::string* dest);

void RecoverKernelEvent(const std::string& src, std::string* dest);

void TimestampToKernelEvent(const std::string& src, std::string* dest);

}  // namespace trace_util

}  // namespace qcraft

#endif  // ONBOARD_GLOBAL_TRACE_UTIL_H_
