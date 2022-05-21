load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "long_term_behavior_estimator",
    srcs = ["long_term_behavior_estimator.cc"],
    hdrs = ["long_term_behavior_estimator.h"],
    deps = [
        "//onboard/prediction/container:object_history",
        "//onboard/proto:prediction_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
