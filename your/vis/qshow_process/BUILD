load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "fen_track",
    hdrs = ["fen_track.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/utils:history_buffer",
        "@boost//:math",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "fen_tracker",
    srcs = ["fen_tracker.cc"],
    hdrs = ["fen_tracker.h"],
    deps = [
        ":fen_track",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:util",
        "//onboard/perception:type_util",
        "@boost//:math",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cpplint()
