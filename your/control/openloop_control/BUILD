load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "openloop_control",
    srcs = [
        "openloop_control.cc",
    ],
    hdrs = [
        "openloop_control.h",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/controllers:controller_util",
        "//onboard/control/openloop_control/proto:openloop_cmd_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "openloop_control_test",
    srcs = ["openloop_control_test.cc"],
    data = glob([
        "testdata/**",
    ]),
    deps = [
        ":openloop_control",
        "//onboard/control:control_defs",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
