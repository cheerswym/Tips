load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "receiver",
    # TODO(Jiaming): temporary workaround
    tags = ["no-dwyu"],
    deps = [
        ":grpc_receiver",
        ":grpc_stream_receiver",
    ],
)

cc_library(
    name = "grpc_receiver",
    srcs = [
        "grpc_receiver.cc",
    ],
    hdrs = ["grpc_receiver.h"],
    deps = [
        ":receiver_interface",
    ],
)

cc_library(
    name = "grpc_stream_receiver",
    srcs = [
        "grpc_stream_receiver.cc",
    ],
    hdrs = ["grpc_stream_receiver.h"],
    deps = [":receiver_interface"],
)

cc_library(
    name = "receiver_interface",
    srcs = [
        "receiver.cc",
    ],
    hdrs = [
        "receiver.h",
        "receiver_context.h",
    ],
    deps = [
        "//onboard/camera/utils:camera_image",
        "//onboard/global:trace",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/transport/proto:node_service_cc_grpc",
        "//onboard/node:node_query_struct",
        "//onboard/node:node_util",
        "//onboard/params/utils:param_util",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

cc_library(
    name = "receiver_test",
    srcs = [
        "grpc_receiver.cc",
        "grpc_stream_receiver.cc",
        "receiver.cc",
    ],
    hdrs = [
        "grpc_receiver.h",
        "grpc_stream_receiver.h",
        "receiver.h",
        "receiver_context.h",
    ],
    copts = [
        "-fno-inline",
    ],
    deps = [
        "//onboard/camera/utils:camera_image",
        "//onboard/global:trace",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/transport/proto:node_service_cc_grpc",
        "//onboard/node:node_query_struct",
        "//onboard/node:node_util",
        "//onboard/params/utils:param_util",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_grpc_grpc//:grpc++",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "grpc_receiver_test",
    srcs = ["grpc_receiver_test.cc"],
    copts = [
        "-fno-inline",
    ],
    deps = [
        ":receiver_test",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@cpp_free_mock",
    ],
)

cc_test(
    name = "grpc_stream_receiver_test",
    srcs = ["grpc_stream_receiver_test.cc"],
    copts = [
        "-fno-inline",
    ],
    deps = [
        ":receiver_test",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@cpp_free_mock",
    ],
)

cpplint()
