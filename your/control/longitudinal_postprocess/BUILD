load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lon_postprocess",
    srcs = ["lon_postprocess.cc"],
    hdrs = ["lon_postprocess.h"],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control/calibration:calibration_manager",
        "//onboard/control/closed_loop_acc:speed_mode_manager",
        "//onboard/control/proto:controller_msg_cc_proto",
        "//onboard/global:car_common",
        "//onboard/math/filters:mean_filter",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cc_test(
    name = "lon_postprocess_test",
    srcs = ["lon_postprocess_test.cc"],
    deps = [
        ":lon_postprocess",
        "//onboard/global:test_main",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
