load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//build_tools/build_rules:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "segment_matcher",
    srcs = [
        "aabox_info.cc",
        "segment_matcher.cc",
        "segment_matcher_kdtree.cc",
    ],
    hdrs = [
        "aabox_info.h",
        "segment_matcher.h",
        "segment_matcher_common.hpp",
        "segment_matcher_kdtree.h",
    ],
    deps = [
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:aabox_kdtree2d",
        "//onboard/math/geometry:segment2d",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "segment_matcher_test",
    srcs = ["segment_matcher_test.cc"],
    deps = [
        ":segment_matcher",
        "//onboard/global:test_main",
        "//onboard/lite:check",
    ],
)

cpplint()
