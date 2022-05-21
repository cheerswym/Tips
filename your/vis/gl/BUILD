load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "gl_common",
    hdrs = ["gl_common.h"],
    deps = [
        "//onboard/lite:check",
        "//third_party/glad",
        "@com_github_google_glog//:glog",
        "@opengl",
    ],
)

cc_library(
    name = "shader",
    srcs = ["shader.cc"],
    hdrs = ["shader.h"],
    deps = [
        ":gl_common",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "text_renderer",
    srcs = ["text_renderer.cc"],
    hdrs = ["text_renderer.h"],
    copts = [
        "-fPIC",
    ],
    data = glob([
        "data/*.png",
    ]),
    deps = [
        ":gl_common",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/vis/common:color",
        "@com_github_google_glog//:glog",
        "@qt//:qt_gui",
    ],
)

cpplint()
