load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ntrip_stream",
    srcs = ["ntrip_stream.cc"],
    hdrs = ["ntrip_stream.h"],
    deps = [
        ":stream",
        ":tcp_stream",
        "//onboard/ap_common/util:string_util",
        "//onboard/lite:logging",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "serial_stream",
    srcs = ["serial_stream.cc"],
    hdrs = ["serial_stream.h"],
    deps = [
        ":stream",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "tcp_stream",
    srcs = ["tcp_stream.cc"],
    hdrs = ["tcp_stream.h"],
    deps = [
        ":stream",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "udp_stream",
    srcs = ["udp_stream.cc"],
    hdrs = ["udp_stream.h"],
    deps = [
        ":stream",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "socketcan_stream",
    srcs = ["socketcan_stream.cc"],
    hdrs = ["socketcan_stream.h"],
    deps = [
        ":stream",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "stream",
    hdrs = ["stream.h"],
    deps = [
        "//onboard/base",
        "//onboard/global:singleton",
    ],
)

cc_library(
    name = "stream_factory",
    srcs = ["stream_factory.cc"],
    hdrs = ["stream_factory.h"],
    deps = [
        ":ntrip_stream",
        ":serial_stream",
        ":socketcan_stream",
        ":stream",
        ":tcp_stream",
        ":udp_stream",
    ],
)

cpplint()
