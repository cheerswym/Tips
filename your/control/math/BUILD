load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "mpc",
    srcs = [
        "mpc_solver.cc",
    ],
    hdrs = [
        "mpc_solver.h",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math/qp:osqp_solver",
        "//onboard/math/qp:qpoases_solver",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "mpc_solver_test",
    size = "small",
    srcs = [
        "mpc_solver_test.cc",
    ],
    deps = [
        ":mpc",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)
