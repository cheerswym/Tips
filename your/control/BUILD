load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "vehicle_control_module",
    srcs = ["vehicle_control_module.cc"],
    hdrs = ["vehicle_control_module.h"],
    deps = [
        ":control_defs",
        ":control_flags",
        ":control_history_state_manager",
        ":control_validation",
        ":controller_agent",
        ":steering_protection",
        ":vehicle_state_interface",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/control/control_check:wire_control_check",
        "//onboard/control/controllers:control_tracking_statistics",
        "//onboard/control/controllers:controller_util",
        "//onboard/control/controllers:steer_calibration",
        "//onboard/control/openloop_control",
        "//onboard/control/parameter_identification",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:counter",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:piecewise_linear_function",
        "//onboard/params:param_manager",
        "//onboard/positioning:positioning_flags",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:control_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "control_history_state_manager",
    srcs = ["control_history_state_manager.cc"],
    hdrs = [
        "control_history_state_manager.h",
    ],
    deps = [
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "controller_agent",
    srcs = [
        "controller_agent.cc",
    ],
    hdrs = [
        "controller_agent.h",
    ],
    deps = [
        ":control_flags",
        ":control_history_state_manager",
        "//onboard/control/controllers:controller_base",
        "//onboard/control/controllers:pole_placement_controller",
        "//onboard/control/controllers:tob_tspkmpc_controller",
        "//onboard/control/controllers:ts_pkmpc_controller",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/global:clock",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "control_flags",
    srcs = ["control_flags.cc"],
    hdrs = ["control_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "steering_protection",
    srcs = ["steering_protection.cc"],
    hdrs = ["steering_protection.h"],
    deps = [
        ":control_defs",
        ":control_history_state_manager",
        ":vehicle_state_interface",
        "//offboard/vis/vantage/charts:chart_util",
        "//onboard/control/controllers:controller_util",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "@boost//:circular_buffer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "control_defs",
    hdrs = ["control_defs.h"],
    deps = [
        "//onboard/math:util",
    ],
)

cc_library(
    name = "trajectory_interface",
    srcs = [
        "trajectory_interface.cc",
    ],
    hdrs = [
        "trajectory_interface.h",
    ],
    deps = [
        ":control_flags",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/control:control_defs",
        "//onboard/eval:qevent",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:trajectory_util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "vehicle_state_interface",
    srcs = [
        "vehicle_state_interface.cc",
    ],
    hdrs = [
        "vehicle_state_interface.h",
    ],
    deps = [
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/control/controllers:controller_util",
        "//onboard/control/proto:vehicle_state_cc_proto",
        "//onboard/lite:check",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "control_validation",
    srcs = [
        "control_validation.cc",
    ],
    hdrs = [
        "control_validation.h",
    ],
    deps = [
        ":control_defs",
        ":control_history_state_manager",
        ":trajectory_interface",
        ":vehicle_state_interface",
        "//onboard/control/controllers:controller_util",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)
###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "steering_protection_test",
    srcs = ["steering_protection_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":control_defs",
        ":steering_protection",
        "//onboard/control/controllers:controller_util",
        "//onboard/global:test_main",
        "//onboard/math:util",
        "//onboard/params:param_manager",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "trajectory_interface_test",
    srcs = ["trajectory_interface_test.cc"],
    deps = [
        ":trajectory_interface",
        "//onboard/control:control_defs",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "control_history_state_manager_test",
    srcs = ["control_history_state_manager_test.cc"],
    deps = [
        ":control_history_state_manager",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "control_validation_test",
    srcs = ["control_validation_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":control_defs",
        ":control_validation",
        ":steering_protection",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
