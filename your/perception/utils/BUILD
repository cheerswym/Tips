load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_filtering_utils",
    srcs = [
        "point_filtering_utils.cc",
    ],
    hdrs = [
        "point_filtering_utils.h",
    ],
    deps = [
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/perception:laser_point",
    ],
)

cc_library(
    name = "vis_utils",
    srcs = [
        "vis_utils.cc",
    ],
    hdrs = [
        "vis_utils.h",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "timer",
    hdrs = [
        "timer.h",
    ],
    deps = [
        "//onboard/global:clock",
        "@com_google_absl//absl/time",
    ],
)

cc_test(
    name = "point_filtering_utils_test",
    srcs = ["point_filtering_utils_test.cc"],
    deps = [
        ":point_filtering_utils",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sensor_view_utils",
    srcs = [
        "sensor_view_utils.cc",
    ],
    hdrs = [
        "sensor_view_utils.h",
    ],
    deps = [
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cpplint()
