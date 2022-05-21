load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_mode_manager",
    srcs = [
        "speed_mode_manager.cc",
    ],
    hdrs = [
        "speed_mode_manager.h",
    ],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cpplint()
