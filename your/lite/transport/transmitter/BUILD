load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "transmitter",
    tags = ["no-dwyu"],
    deps = [
        ":grpc_stream_transmitter",
        ":grpc_transmitter",
    ],
)

cc_library(
    name = "grpc_transmitter",
    srcs = [
        "grpc_transmitter.cc",
    ],
    hdrs = ["grpc_transmitter.h"],
    deps = [
        ":transmitter_interface",
        "//onboard/global:clock",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "grpc_stream_transmitter",
    srcs = [
        "grpc_stream_transmitter.cc",
    ],
    hdrs = ["grpc_stream_transmitter.h"],
    deps = [
        ":transmitter_interface",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "transmitter_interface",
    srcs = ["transmitter.cc"],
    hdrs = ["transmitter.h"],
    deps = [
        "//onboard/global:car_common",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/transport/proto:node_service_cc_grpc",
        "//onboard/node:node_query_struct",
        "//onboard/node:node_util",
        "//onboard/params/utils:param_util",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/synchronization",
    ],
)

# Functions that need to be mocked in the test case are not inlined
cc_library(
    name = "transmitter_test",
    srcs = [
        "grpc_stream_transmitter.cc",
        "grpc_transmitter.cc",
        "transmitter.cc",
    ],
    hdrs = [
        "grpc_stream_transmitter.h",
        "grpc_transmitter.h",
        "transmitter.h",
    ],
    copts = [
        "-fno-inline",
    ],
    deps = [
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/transport/proto:node_service_cc_grpc",
        "//onboard/node:node_query_struct",
        "//onboard/node:node_util",
        "//onboard/params/utils:param_util",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/synchronization",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "grpc_stream_transmitter_test",
    srcs = [
        "grpc_stream_transmitter_test.cc",
    ],
    copts = [
        "-fno-inline",
    ],
    deps = [
        ":transmitter_test",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@cpp_free_mock",
    ],
)

cc_test(
    name = "grpc_transmitter_test",
    srcs = [
        "grpc_transmitter_test.cc",
    ],
    copts = [
        "-fno-inline",
    ],
    deps = [
        ":transmitter_test",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@cpp_free_mock",
    ],
)

cpplint()
