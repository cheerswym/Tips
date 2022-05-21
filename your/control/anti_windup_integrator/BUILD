load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "anti_windup_integrator",
    srcs = [
        "anti_windup_integrator.cc",
    ],
    hdrs = [
        "anti_windup_integrator.h",
    ],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/lite:logging",
    ],
)

cpplint()
