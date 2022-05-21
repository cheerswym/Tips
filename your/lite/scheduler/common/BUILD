load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "cpu_affinity",
    srcs = ["cpu_affinity.cc"],
    hdrs = ["cpu_affinity.h"],
    deps = [
        "//onboard/lite:check",
        "@com_github_google_glog//:glog",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
