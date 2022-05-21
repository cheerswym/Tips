load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wire_control_check",
    srcs = ["wire_control_check.cc"],
    hdrs = ["wire_control_check.h"],
    deps = [
        "//onboard/control/controllers:controller_util",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math/filters:mean_filter",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "wire_control_check_test",
    srcs = ["wire_control_check_test.cc"],
    deps = [
        ":wire_control_check",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
