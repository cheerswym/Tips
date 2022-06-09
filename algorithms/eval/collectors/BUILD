load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "qevent_service_collector",
    srcs = ["qevent_service_collector.cc"],
    hdrs = ["qevent_service_collector.h"],
    deps = [
        "//offboard/dashboard/utils:retry",
        "//offboard/eval/proto:qevents_service_cc_grpc",
        "//onboard/async:async_util",
        "//onboard/eval/collectors:qevent_collector",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/cleanup",
    ],
)

cc_library(
    name = "qevent_file_collector",
    srcs = ["qevent_file_collector.cc"],
    hdrs = ["qevent_file_collector.h"],
    deps = [
        "//onboard/async:async_util",
        "//onboard/eval/collectors:qevent_collector",
        "//onboard/utils:file_util",
    ],
)

cpplint()
