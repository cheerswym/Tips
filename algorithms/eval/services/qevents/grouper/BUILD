load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
cc_library(
    name = "qevents_grouper_base",
    hdrs = ["qevents_grouper_base.h"],
    deps = [
        "//offboard/eval/proto:qevents_service_cc_grpc",
        "//onboard/global:logging",
        "//onboard/global:registry",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_grpc_grpc//:grpc++_reflection",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "timestamp_grouper",
    srcs = ["timestamp_grouper.cc"],
    hdrs = ["timestamp_grouper.h"],
    deps = [
        ":qevents_grouper_base",
        "//onboard/base",
    ],
)

cc_library(
    name = "qevents_grouper",
    srcs = ["qevents_grouper.cc"],
    hdrs = ["qevents_grouper.h"],
    deps = [
        ":qevents_grouper_base",
        ":timestamp_grouper",
        "//onboard/base",
    ],
)

# Tests.
cc_test(
    name = "qevents_grouper_test",
    srcs = [
        "qevents_grouper_test.cc",
    ],
    deps = [
        ":qevents_grouper",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "timestamp_grouper_test",
    srcs = [
        "timestamp_grouper_test.cc",
    ],
    deps = [
        ":timestamp_grouper",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
