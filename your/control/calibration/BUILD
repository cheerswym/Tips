load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "calibration_manager",
    srcs = ["calibration_manager.cc"],
    hdrs = ["calibration_manager.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:interpolation_2d",
        "//onboard/math:piecewise_linear_function",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "control_calibration_module",
    srcs = ["control_calibration_module.cc"],
    hdrs = ["control_calibration_module.h"],
    deps = [
        "//onboard/control/calibration/calibration_tools",
        "//onboard/lite:lite_module",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
    ],
)

cc_test(
    name = "calibration_manager_test",
    srcs = ["calibration_manager_test.cc"],
    deps = [
        ":calibration_manager",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
