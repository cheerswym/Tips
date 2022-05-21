load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "canvas",
    srcs = ["canvas.cc"],
    hdrs = ["canvas.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:util",
        "//onboard/vis/canvas/proto:canvas_buffer_cc_proto",
        "//onboard/vis/common:color",
        "//onboard/vis/gl:gl_common",
        "//onboard/vis/gl:text_renderer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "canvas_test",
    srcs = ["canvas_test.cc"],
    copts = [
        "-Wno-thread-safety-analysis",
    ],
    deps = [
        ":canvas",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
