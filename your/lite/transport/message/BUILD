load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "message_hub",
    hdrs = ["message_hub.h"],
    deps = [
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "inner_message_hub",
    srcs = ["inner_message_hub.cc"],
    hdrs = [
        "inner_message_hub.h",
    ],
    deps = [
        ":message_hub",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "shm_message_hub",
    srcs = ["shm_message_hub.cc"],
    hdrs = [
        "shm_message_hub.h",
    ],
    deps = [
        ":message_hub",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:counter",
        "//onboard/lite:check",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:logging",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/transport/shm:condition_notifier",
        "//onboard/lite/transport/shm:lite_shm_header",
        "//onboard/lite/transport/shm:shm_debug_util",
        "//onboard/lite/transport/shm:shm_transmitter",
        "//onboard/utils:thread_util",
        "@com_google_absl//absl/synchronization",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
