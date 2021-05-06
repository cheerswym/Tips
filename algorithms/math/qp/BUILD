load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//build_tools/build_rules:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "dense_qp_solver",
    hdrs = ["dense_qp_solver.h"],
    deps = [
        "//onboard/base",
        "//onboard/math:eigen",
    ],
)

cc_library(
    name = "qpoases_solver",
    srcs = ["qpoases_solver.cc"],
    hdrs = ["qpoases_solver.h"],
    # NOTE(jiaming): to suppress qpOASES build warnings
    copts = ["-Wno-overloaded-virtual"],
    linkstatic = False,
    deps = [
        ":dense_qp_solver",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "@qpOASES",
    ],
)

cc_library(
    name = "sparse_qp_solver",
    hdrs = ["sparse_qp_solver.h"],
    deps = [
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:eigen",
    ],
)

cc_library(
    name = "osqp_solver",
    srcs = ["osqp_solver.cc"],
    hdrs = ["osqp_solver.h"],
    linkstatic = False,
    deps = [
        ":sparse_qp_solver",
        "//onboard/lite:check",
        "@osqp",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "qpoases_solver_test",
    srcs = ["qpoases_solver_test.cc"],
    deps = [
        ":qpoases_solver",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "osqp_solver_test",
    srcs = ["osqp_solver_test.cc"],
    deps = [
        ":osqp_solver",
        "//onboard/global:test_main",
    ],
)

cpplint()
