load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "parameter_identification",
    srcs = [
        "parameter_identification.cc",
    ],
    hdrs = [
        "parameter_identification.h",
    ],
    deps = [
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/control:control_history_state_manager",
        "//onboard/control/controllers:controller_util",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "parameter_identification_test",
    srcs = ["parameter_identification_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":parameter_identification",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
