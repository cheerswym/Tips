load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "transport_factory",
    srcs = [
        "transport_factory.cc",
    ],
    hdrs = ["transport_factory.h"],
    deps = [
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/transport/receiver",
        "//onboard/lite/transport/receiver:grpc_receiver",
        "//onboard/lite/transport/receiver:grpc_stream_receiver",
        "//onboard/lite/transport/receiver:receiver_interface",
        "//onboard/lite/transport/receiver:receiver_test",
        "//onboard/lite/transport/transmitter",
        "//onboard/lite/transport/transmitter:grpc_stream_transmitter",
        "//onboard/lite/transport/transmitter:grpc_transmitter",
        "//onboard/lite/transport/transmitter:transmitter_interface",
        "//onboard/lite/transport/transmitter:transmitter_test",
        "//onboard/params:vehicle_param_api",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "transport_factory_test",
    srcs = ["transport_factory_test.cc"],
    deps = [
        ":transport_factory",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
