#include "onboard/global/trace_util.h"

#include <algorithm>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/strings/numbers.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "glog/logging.h"
#include "nlohmann/json.hpp"
#include "zlib/zlib.h"

namespace qcraft {

namespace {

constexpr int kRecoverSize = 10000 * 200;
constexpr int kTimestampSize = 100000 * 24;
constexpr char kTracingMarkWrite[] = "tracing_mark_write";
constexpr char kMarkTrace[] = "MarkTrace";
constexpr char kSystemProcess[] = "#P:";
constexpr int kTaskPidIndex = 0;
constexpr int kTimestampIndex = 3;
constexpr int kTracingMarkIndex = 5;
constexpr int kMarkTraceTimestampIndex = 3;

std::string GetRecoverTracingRecord(const absl::string_view& tracing_record,
                                    const absl::string_view& source,
                                    const absl::string_view& target) {
  std::string record(tracing_record.data(), tracing_record.size());
  record.replace(record.rfind(std::string(source.data(), source.size())),
                 source.size(), target);
  return record;
}

}  // namespace

namespace trace_util {

std::string TraceProtoToJson(const TraceProto& trace_proto) {
  auto top_level_json = nlohmann::json::array();
  for (const auto& meta_event_proto : trace_proto.meta_event()) {
    if (meta_event_proto.has_process_name()) {
      // create process_name meta event
      nlohmann::json json_obj;
      json_obj["name"] = "process_name";
      json_obj["ph"] = "M";
      json_obj["pid"] = meta_event_proto.process_id();
      json_obj["tid"] = meta_event_proto.thread_id();

      nlohmann::json json_args_obj;
      json_args_obj["name"] = meta_event_proto.process_name();
      json_obj["args"] = json_args_obj;
      top_level_json.push_back(json_obj);
    }
  }

  // Fix the trace timestamp overlapping issue.
  for (const auto& trace_events : trace_proto.trace()) {
    std::vector<TraceEventProto> sorted_trace_events;
    for (const auto& trace_event : trace_events.trace_events()) {
      sorted_trace_events.push_back(trace_event);
    }
    std::sort(sorted_trace_events.begin(), sorted_trace_events.end(),
              [](const auto& te1, const auto& te2) {
                const int64_t start1 = static_cast<int64_t>(te1.start() * 1e6);
                const int64_t end1 = static_cast<int64_t>(te1.end() * 1e6);
                const int64_t start2 = static_cast<int64_t>(te2.start() * 1e6);
                const int64_t end2 = static_cast<int64_t>(te2.end() * 1e6);
                return std::make_pair(start1, -end1) <
                       std::make_pair(start2, -end2);
              });

    int num_identical_start = 0;
    for (int i = 1; i < sorted_trace_events.size(); ++i) {
      auto& trace_event = sorted_trace_events[i];
      if (static_cast<int64_t>(trace_event.start() * 1e6) ==
          static_cast<int64_t>(sorted_trace_events[i - 1].start() * 1e6)) {
        num_identical_start++;
        trace_event.set_start(trace_event.start() + 1e-6 * num_identical_start);
      } else {
        num_identical_start = 0;
      }
      if (static_cast<int64_t>(trace_event.end() * 1e6) <=
          static_cast<int64_t>(trace_event.start() * 1e6)) {
        trace_event.set_end(trace_event.start() + 1e-6);
      }
      // template obj
      nlohmann::json json_base_obj;
      json_base_obj["cat"] = "renderer";
      json_base_obj["pid"] = trace_events.process_id();
      json_base_obj["tid"] = trace_events.thread_id();
      json_base_obj["name"] = trace_event.name();

      // create args
      nlohmann::json json_obj_args;
      for (const auto& arg : trace_event.args()) {
        int int_val;
        double float_val;
        if (absl::SimpleAtoi(arg.value(), &int_val)) {
          json_obj_args[arg.key()] = int_val;
        } else if (absl::SimpleAtod(arg.value(), &float_val)) {
          json_obj_args[arg.key()] = float_val;
        } else {
          json_obj_args[arg.key()] = arg.value();
        }
      }
      json_base_obj["args"] = json_obj_args;

      nlohmann::json begin_json_obj = json_base_obj;
      begin_json_obj["ts"] = trace_event.start() * 1e6;
      begin_json_obj["ph"] = "B";

      nlohmann::json end_json_obj = json_base_obj;
      end_json_obj["ts"] = trace_event.end() * 1e6;
      end_json_obj["ph"] = "E";

      top_level_json.push_back(begin_json_obj);
      top_level_json.push_back(end_json_obj);
    }
  }
  return top_level_json.dump();
}

std::string TraceProtoToKernelEvent(const TraceProto& trace_proto) {
  if (!trace_proto.has_kernel_event()) {
    return "";
  }

  return trace_proto.kernel_event();
}

bool Compress(const std::string& src, std::string* dest) {
  if (dest == nullptr) {
    return false;
  }

  static const float kCompressRate = 0.1;
  dest->resize(src.size() * kCompressRate + 1);

  z_stream stream = {0};
  stream.avail_in = src.size();
  stream.avail_out = dest->size();
  stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(src.data()));
  stream.next_out = reinterpret_cast<Bytef*>(const_cast<char*>(dest->data()));
  int ret = deflateInit(&stream, Z_DEFAULT_COMPRESSION);
  // int ret = deflateInit2(&strm, Z_DEFAULT_COMPRESSION, Z_DEFLATED,
  //                        MAX_WBITS + 16, MAX_MEM_LEVEL, Z_DEFAULT_STRATEGY);
  if (ret == Z_OK) {
    ret = deflate(&stream, Z_FINISH);
    deflateEnd(&stream);
    if (ret == Z_STREAM_END) {
      dest->resize(dest->size() - stream.avail_out);
      return true;
    }
  }

  return false;
}

bool Decompress(const std::string& src, std::string* dest) {
  if (dest == nullptr) {
    return false;
  }

  static const int kDecompressRate = 50;
  dest->resize(src.size() * kDecompressRate);

  z_stream stream = {0};
  stream.avail_in = src.size();
  stream.avail_out = dest->size();
  stream.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(src.data()));
  stream.next_out = reinterpret_cast<Bytef*>(const_cast<char*>(dest->data()));
  int ret = inflateInit(&stream);
  // int ret = inflateInit2(&stream, MAX_WBITS + 16);
  if (ret == Z_OK) {
    ret = inflate(&stream, Z_FINISH);
    inflateEnd(&stream);
    if (ret == Z_STREAM_END) {
      dest->resize(stream.total_out);
      return true;
    }
  }

  return false;
}

// #           TASK-PID   CPU#  ||||    TIMESTAMP  FUNCTION
// #              | |       |   ||||       |         |
//  Q-NodeStateModu-26046 [001] ...1  6022.834248: tracing_mark_write:
//  trace_event_clock_sync: parent_ts=6022.834473
void RecoverKernelEvent(const std::string& src, std::string* dest) {
  dest->reserve(src.size() + kRecoverSize);

  int system_process_count = 0;
  std::unordered_set<int> system_process;
  absl::string_view lastest_thread_id;
  absl::string_view lastest_tracing_mark;
  absl::string_view lastest_tracing_record;
  std::unordered_map<absl::string_view, std::set<absl::string_view>>
      tracing_marks;
  std::unordered_map<absl::string_view, std::stack<absl::string_view>>
      tracing_records;
  for (absl::string_view tracing_record : absl::StrSplit(src, '\n')) {
    if (tracing_record[0] == '#') {
      if (system_process_count == 0) {
        // find system process tag that it was marked by ftrace
        if (tracing_record.find(kSystemProcess) != absl::string_view::npos) {
          // # entries-in-buffer/entries-written: 497761/93319786   #P:16
          const std::vector<absl::string_view> items =
              absl::StrSplit(tracing_record, ":", absl::SkipWhitespace());
          system_process_count =
              std::stoi(std::string(items[items.size() - 1]));
          system_process.reserve(system_process_count);
        }
      }
    } else {
      // remove tracing mark if we don't collect one time tracing record per cpu
      const auto skip_tracing_mark_write =
          (system_process.size() < system_process_count);
      if (skip_tracing_mark_write) {
        const auto index = tracing_record.find("[");
        if (index != absl::string_view::npos) {
          const auto cpu_id =
              std::stoi(std::string(tracing_record.substr(index + 1, 3)));
          system_process.insert(cpu_id);
        }
      }

      // find tracing mark tag that it was created by scoped_trace
      if (tracing_record.find(kTracingMarkWrite) != absl::string_view::npos) {
        if (skip_tracing_mark_write) {
          continue;
        }

        const std::vector<absl::string_view> items =
            absl::StrSplit(tracing_record, " ", absl::SkipWhitespace());
        const std::vector<absl::string_view> task_pid =
            absl::StrSplit(items[kTaskPidIndex], "-");
        const auto& thread_id = task_pid[task_pid.size() - 1];
        const auto& tracing_mark = items[kTracingMarkIndex];
        if (tracing_records.count(thread_id)) {
          if (tracing_mark[0] == 'B') {
            tracing_marks[thread_id].insert(tracing_mark);
            tracing_records[thread_id].push(tracing_mark);
          } else if (tracing_mark[0] == 'E') {
            // drop event when it has redundant tag 'E'
            if (tracing_records[thread_id].size() == 0) {
              continue;
            } else {
              tracing_marks[thread_id].erase(tracing_records[thread_id].top());
              tracing_records[thread_id].pop();
            }
          }
        } else {
          std::set<absl::string_view> marks;
          marks.insert(tracing_mark);
          tracing_marks[thread_id] = marks;

          std::stack<absl::string_view> records;
          records.push(tracing_mark);
          tracing_records[thread_id] = records;
        }

        lastest_thread_id = thread_id;
        lastest_tracing_mark = tracing_mark;
        lastest_tracing_record = tracing_record;
      }
    }

    dest->append(tracing_record.data(), tracing_record.size());
    dest->append("\n");
  }

  // add event when it doesn't finsh that function call
  const auto recover_tracing_record = GetRecoverTracingRecord(
      lastest_tracing_record, lastest_tracing_mark, "E");
  for (const auto& tracing_record : tracing_records) {
    dest->append(GetRecoverTracingRecord(
        recover_tracing_record, lastest_thread_id, tracing_record.first));
    dest->append("\n");
  }

  // we don't do any recover if we can't find all cpu tracing record
  if (system_process.size() < system_process_count) {
    *dest = src;
  }
}

// Q-NodeStateModu-22605 [006] ...1  7186.964084: tracing_mark_write:
// B|22585|MarkTrace|timestamp=1629970090035
void TimestampToKernelEvent(const std::string& src, std::string* dest) {
  dest->reserve(src.size() + kTimestampSize);

  double marktrace_tick = 0;
  double marktrace_timestamp = 0;
  std::vector<absl::string_view> tracing_records = absl::StrSplit(src, '\n');
  auto it = tracing_records.rbegin();
  while (it != tracing_records.rend()) {
    if (it->rfind(kMarkTrace) != absl::string_view::npos) {
      const std::vector<absl::string_view> items =
          absl::StrSplit(*it, " ", absl::SkipWhitespace());
      const std::vector<absl::string_view> marktrace_items =
          absl::StrSplit(items[kTracingMarkIndex], "|");
      const std::vector<absl::string_view> timestamp_items =
          absl::StrSplit(marktrace_items[kMarkTraceTimestampIndex], "=");
      marktrace_tick = std::stod(std::string(items[kTimestampIndex]));
      marktrace_timestamp = std::stod(std::string(timestamp_items[1])) / 1000;
      break;
    }
    ++it;
  }

  if (marktrace_tick == 0 || marktrace_timestamp == 0) {
    *dest = src;
    return;
  }

  for (auto tracing_record : tracing_records) {
    // find tracing mark tag that it was created by scoped_trace
    if (tracing_record.find(kTracingMarkWrite) != absl::string_view::npos) {
      const std::vector<absl::string_view> items =
          absl::StrSplit(tracing_record, " ", absl::SkipWhitespace());
      const auto& tracing_mark = items[kTracingMarkIndex];
      if (tracing_mark[0] == 'B') {
        std::string append;
        const auto current_tick =
            std::stod(std::string(items[kTimestampIndex]));
        const auto current_timestamp =
            marktrace_timestamp + (current_tick - marktrace_tick);

        if (tracing_mark.rfind('=') == absl::string_view::npos) {
          append = absl::StrFormat("|timestamp=%.6f s", current_timestamp);
        } else {
          append = absl::StrFormat(";timestamp=%.6f s", current_timestamp);
        }

        dest->append(std::string(tracing_record) + append);
        dest->append("\n");

        continue;
      }
    }

    dest->append(tracing_record.data(), tracing_record.size());
    dest->append("\n");
  }
}

}  // namespace trace_util
}  // namespace qcraft
