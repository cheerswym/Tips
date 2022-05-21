load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hmm",
    srcs = ["hmm.cc"],
    hdrs = ["hmm.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:eigen",
    ],
)

cc_test(
    name = "hmm_test",
    srcs = ["hmm_test.cc"],
    deps = [
        ":hmm",
        "//onboard/global:logging",
        "//onboard/global:test_main",
    ],
)

cpplint()
