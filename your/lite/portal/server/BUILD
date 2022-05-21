load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "run_event_process",
    srcs = ["run_event_process.cc"],
    hdrs = ["run_event_process.h"],
    deps = [
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/lite/portal/proto:portal_service_cc_proto",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "portal_service_server",
    srcs = ["portal_service_server.cc"],
    hdrs = ["portal_service_server.h"],
    deps = [
        ":run_event_process",
        "//onboard/global:clock",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/lite/portal/proto:portal_service_cc_grpc",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/proto:q_state_cc_proto",
        "//onboard/proto:q_vehicle_info_cc_proto",
        "//onboard/v2x:v2x_util",
        "@com_github_google_glog//:glog",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Binaries.
cc_binary(
    name = "portal_service_server_main",
    srcs = ["portal_service_server_main.cc"],
    copts = [
        "-fPIC",
    ],
    deps = [
        ":portal_service_server",
        "//onboard/global:init_qcraft",
        "//onboard/proto:port_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/strings",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "run_event_process_test",
    srcs = ["run_event_process_test.cc"],
    deps = [
        ":run_event_process",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
