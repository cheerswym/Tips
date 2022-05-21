#ifndef ONBOARD_UTILS_TEST_UTIL_H_
#define ONBOARD_UTILS_TEST_UTIL_H_

#include <optional>
#include <string>

#include "gmock/gmock.h"
#include "google/protobuf/text_format.h"
#include "onboard/utils/proto_util.h"

namespace qcraft {

// Compares if two protobuf messages are equivalent.
MATCHER_P(ProtoEq, proto, "") {
  const auto diff = ProtoDiff(arg, proto, /*partial=*/false);
  *result_listener << "Actual: " << arg.DebugString()
                   << "\nExpected: " << proto.DebugString();
  if (diff.has_value()) {
    *result_listener << "\nDifference: " << diff.value();
  }
  return !diff.has_value();
}

// Compares if a protobuf message is equivalent to provided text.
MATCHER_P(ProtoEqText, text_proto, "") {
  using ArgType = typename std::remove_cv<
      typename std::remove_reference<decltype(arg)>::type>::type;
  ArgType expected_proto;
  google::protobuf::TextFormat::ParseFromString(text_proto, &expected_proto);
  const auto diff = ProtoDiff(arg, expected_proto, /*partial=*/false);
  *result_listener << "Actual: " << arg.DebugString()
                   << "\nExpected: " << text_proto;
  if (diff.has_value()) {
    *result_listener << "\nDifference: " << diff.value();
  }
  return !diff.has_value();
}

// Check if the provided proto partially matches.
MATCHER_P(ProtoPartialMatches, proto, "") {
  const auto diff = ProtoDiff(proto, arg, /*partial=*/true);
  *result_listener << "Actual: " << arg.DebugString()
                   << "\nExpected: " << proto.DebugString();
  if (diff.has_value()) {
    *result_listener << "\nDifference: " << diff.value();
  }
  return !diff.has_value();
}

// Check if the provided text partially matches.
MATCHER_P(ProtoPartialMatchesText, partial_text_proto, "") {
  using ArgType = typename std::remove_cv<
      typename std::remove_reference<decltype(arg)>::type>::type;
  ArgType expected_proto;
  google::protobuf::TextFormat::ParseFromString(partial_text_proto,
                                                &expected_proto);
  const auto diff = ProtoDiff(expected_proto, arg, /*partial=*/true);
  *result_listener << "Actual: " << arg.DebugString()
                   << "\nExpected: " << partial_text_proto;
  if (diff.has_value()) {
    *result_listener << "\nDifference: " << diff.value();
  }
  return !diff.has_value();
}

// Check if the provided text partially and the float or double number
// approximately matches the protobuf message.
MATCHER_P2(ProtoPartialApproximatelyMatchesText, partial_text_proto, epsilon,
           "") {
  using ArgType = typename std::remove_cv<
      typename std::remove_reference<decltype(arg)>::type>::type;
  ArgType expected_proto;
  google::protobuf::TextFormat::ParseFromString(partial_text_proto,
                                                &expected_proto);
  const auto diff =
      ProtoDiff(expected_proto, arg, /*partial=*/true, /*margin=*/epsilon);
  *result_listener << "Actual: " << arg.DebugString()
                   << "\nExpected: " << partial_text_proto;
  if (diff.has_value()) {
    *result_listener << "\nDifference: " << diff.value();
  }
  return !diff.has_value();
}

}  // namespace qcraft

#endif  // ONBOARD_UTILS_TEST_UTIL_H_
