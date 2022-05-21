load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "client",
    srcs = [
        "client.cc",
    ],
    hdrs = ["client.h"],
    deps = [
        ":client_base",
        "//onboard/global:counter",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/lite/transport:transport_factory",
        "//onboard/node:node_query_struct",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "client_base",
    hdrs = ["client_base.h"],
)

cc_library(
    name = "server",
    srcs = [
        "server.cc",
    ],
    hdrs = ["server.h"],
    deps = [
        ":server_base",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:spin_util",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:logging",
        "//onboard/lite/transport:transport_factory",
        "//onboard/node:node_query_struct",
        "//onboard/proto:lite_msg_cc_proto",
    ],
)

cc_library(
    name = "server_base",
    hdrs = ["server_base.h"],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
