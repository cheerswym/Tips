load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "ondemand_sidecar",
    srcs = ["ondemand_sidecar.cc"],
    hdrs = ["ondemand_sidecar.h"],
    deps = [
        "//onboard/ap_common/util:blocking_queue",
        "//onboard/lite:lite_module",
        "//onboard/utils:thread_util",
    ],
)

cc_library(
    name = "user_input_sidecar",
    srcs = ["user_input_sidecar.cc"],
    hdrs = ["user_input_sidecar.h"],
    deps = [
        "//onboard/lite:lite_module",
        "//onboard/utils:thread_util",
    ],
)

cc_library(
    name = "periodic_sidecar",
    srcs = ["periodic_sidecar.cc"],
    hdrs = ["periodic_sidecar.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/eval/collectors:qevent_log_collector",
        "//onboard/global:counter",
        "//onboard/global:system_info",
        "//onboard/lite:lite_module",
        "//onboard/utils:periodic_runner",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
