load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "runner",
    hdrs = ["runner.h"],
    deps = [
        "//onboard/eval/collectors:qevent_log_collector",
        "//onboard/lite:lite_client",
        "//onboard/lite:lite_module",
        "//onboard/lite/transport/message:shm_message_hub",
        "@folly",
    ],
)

cc_library(
    name = "runner_flags",
    srcs = [
        "runner_flags.cc",
    ],
    hdrs = ["runner_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "module_runner",
    srcs = [
        "module_runner.cc",
    ],
    hdrs = ["module_runner.h"],
    data = [
        "//offboard/calibration/data:lidar_extrinsics_file",
        "//onboard/lidar/data:lidar_common_params_data",
        "//onboard/lite/launch_config:launch_config_files",
        "//onboard/lite/launch_config:module_config_files",
    ],
    deps = [
        ":runner",
        ":runner_flags",
        "//common/run:release_cc_proto",
        "//onboard/base:base_flags",
        "//onboard/eval/collectors:qevent_collector",
        "//onboard/global:buffered_logger",
        "//onboard/global:car_common",
        "//onboard/global:ftrace",
        "//onboard/global:logging",
        "//onboard/global:registry",
        "//onboard/lite:check",
        "//onboard/lite:lite2_flags",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_transport",
        "//onboard/lite:logging",
        "//onboard/lite:module_config_helper",
        "//onboard/lite:offboard_modules",
        "//onboard/lite:onboard_modules",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/sidecar:ondemand_sidecar",
        "//onboard/lite/sidecar:periodic_sidecar",
        "//onboard/lite/transport/shm:shm_manager",
        "//onboard/logging/proto:lite_run_cc_proto",
        "//onboard/maps:map_selector",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

cc_library(
    name = "mock_runner",
    srcs = [
        "mock_runner.cc",
    ],
    hdrs = ["mock_runner.h"],
    deps = [
        ":runner",
        ":runner_flags",
        "//common/run:release_cc_proto",
        "//onboard/base:base_flags",
        "//onboard/eval/collectors:qevent_collector",
        "//onboard/global:buffered_logger",
        "//onboard/global:car_common",
        "//onboard/global:ftrace",
        "//onboard/global:logging",
        "//onboard/global:registry",
        "//onboard/lite:check",
        "//onboard/lite:lite2_flags",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_transport",
        "//onboard/lite:logging",
        "//onboard/lite:mock_modules",
        "//onboard/lite:module_config_helper",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/sidecar:ondemand_sidecar",
        "//onboard/lite/sidecar:periodic_sidecar",
        "//onboard/lite/transport/shm:shm_manager",
        "//onboard/logging/proto:lite_run_cc_proto",
        "//onboard/maps:map_selector",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

cc_library(
    name = "autonomy_runner",
    srcs = [
        "autonomy_runner.cc",
    ],
    hdrs = ["autonomy_runner.h"],
    deps = [
        ":runner",
        ":runner_flags",
        "//common/run:release_cc_proto",
        "//onboard/autonomy:autonomy_module",
        "//onboard/base:base_flags",
        "//onboard/eval/collectors:qevent_collector",
        "//onboard/global:buffered_logger",
        "//onboard/global:car_common",
        "//onboard/global:ftrace",
        "//onboard/global:logging",
        "//onboard/global:registry",
        "//onboard/lite:check",
        "//onboard/lite:lite2_flags",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_transport",
        "//onboard/lite:logging",
        "//onboard/lite:module_config_helper",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/sidecar:ondemand_sidecar",
        "//onboard/lite/sidecar:periodic_sidecar",
        "//onboard/lite/transport/shm:shm_manager",
        "//onboard/logging/proto:lite_run_cc_proto",
        "//onboard/maps:map_selector",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/rsim/common:utils",
        "//onboard/rsim/config",
        "//onboard/rsim/config/proto:config_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

###############################################################################
# Tests.
# Client test

cpplint()
