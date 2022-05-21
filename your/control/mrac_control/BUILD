load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mrac_control",
    srcs = ["mrac_control.cc"],
    hdrs = ["mrac_control.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control/controllers:controller_util",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math/filters:mean_filter",
        "//onboard/proto:control_cmd_cc_proto",
        "@boost//:circular_buffer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
    ],
)

cc_test(
    name = "mrac_control_test",
    srcs = ["mrac_control_test.cc"],
    data = glob([
        "testdata/**",
    ]),
    deps = [
        ":mrac_control",
        "//onboard/control/controllers:controller_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
