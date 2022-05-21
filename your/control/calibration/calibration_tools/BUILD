load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "calibration_utils",
    srcs = ["calibration_utils.cc"],
    hdrs = ["calibration_utils.h"],
    deps = [
        "//onboard/math:eigen",
    ],
)

cc_library(
    name = "calibration_interface",
    hdrs = ["calibration_interface.h"],
    deps = [
        "//onboard/control/calibration/calibration_tools/proto:control_calibration_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "calibration_idle",
    srcs = ["calibration_idle.cc"],
    hdrs = ["calibration_idle.h"],
    deps = [
        ":calibration_interface",
        ":calibration_utils",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "calibration_slide",
    srcs = ["calibration_slide.cc"],
    hdrs = ["calibration_slide.h"],
    deps = [
        ":calibration_interface",
        ":calibration_utils",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "calibration_force",
    srcs = ["calibration_force.cc"],
    hdrs = ["calibration_force.h"],
    deps = [
        ":calibration_interface",
        ":calibration_utils",
        "//onboard/math:piecewise_linear_function",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "calibration_tools",
    srcs = ["calibration_tools.cc"],
    hdrs = ["calibration_tools.h"],
    deps = [
        ":calibration_force",
        ":calibration_idle",
        ":calibration_interface",
        ":calibration_slide",
        "//offboard/vis/vantage/charts:chart_util",
        "//onboard/math/filters:mean_filter",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/vis/common:color",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "calibration_tools_test",
    srcs = ["calibration_tools_test.cc"],
    data = glob([
        "testdata/**",
    ]),
    deps = [
        ":calibration_tools",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "calibration_utils_test",
    srcs = ["calibration_utils_test.cc"],
    deps = [
        ":calibration_utils",
        "//onboard/global:test_main",
    ],
)

cpplint()
