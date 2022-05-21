load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "node_state_module",
    srcs = ["node_state_module.cc"],
    hdrs = ["node_state_module.h"],
    deps = [
        "//onboard/global:clock",
        "//onboard/global:system_info",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/lite:module_config_helper",
        "//onboard/node/proto:node_state_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:execution_issue_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:periodic_runner",
        "@boost//:process",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "node_module",
    srcs = [
        "node_module.cc",
    ],
    hdrs = ["node_module.h"],
    deps = [
        ":node",
        "//onboard/lite:lite_module",
    ],
)

cc_library(
    name = "node",
    srcs = [
        "node.cc",
    ],
    hdrs = ["node.h"],
    deps = [
        ":node_query_struct",
        ":node_util",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/lite:module_config_helper",
        "//onboard/lite:qissue_trans",
        "//onboard/lite/service:client",
        "//onboard/lite/service:server",
        "//onboard/params:param_manager",
    ],
)

cc_library(
    name = "node_util",
    srcs = [
        "node_util.cc",
    ],
    hdrs = ["node_util.h"],
    data = [
        "test/interactive_playback_with_node_lite2.pb.txt",
    ],
    deps = [
        "//onboard/global:car_common",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/proto:ads_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "node_query_struct",
    hdrs = ["node_query_struct.h"],
)

###############################################################################
# Tests.
# cc_test(
#     name = "node_test",
#     srcs = [
#         "node_test.cc",
#     ],
#     deps = [
#         ":node",
#         "//onboard/conf:run_conf_man",
#         "//onboard/global:test_main",
#         "//onboard/lite:logging",
#         "@com_google_absl//absl/strings:str_format",
#     ],
# )

cpplint()
