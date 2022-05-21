load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controller_base",
    hdrs = ["controller_base.h"],
    deps = [
        "//onboard/control:control_history_state_manager",
        "//onboard/control:steering_protection",
        "//onboard/control:trajectory_interface",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/proto:controller_conf_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "pole_placement_controller",
    srcs = [
        "pole_placement_controller.cc",
    ],
    hdrs = [
        "pole_placement_controller.h",
    ],
    deps = [
        ":controller_base",
        ":controller_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/ap_common/configs/proto:vehicle_config_cc_proto",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:control_history_state_manager",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/anti_windup_integrator",
        "//onboard/control/calibration:calibration_manager",
        "//onboard/control/closed_loop_acc:speed_mode_manager",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/math:mpc",
        "//onboard/control/mrac_control",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:interpolation_2d",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/filters:digital_filter",
        "//onboard/math/filters:digital_filter_coefficients",
        "//onboard/math/filters:mean_filter",
        "//onboard/math/geometry:util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "ts_pkmpc_controller",
    srcs = [
        "ts_pkmpc_controller.cc",
    ],
    hdrs = [
        "ts_pkmpc_controller.h",
    ],
    deps = [
        ":controller_base",
        ":controller_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/ap_common/configs/proto:vehicle_config_cc_proto",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:control_history_state_manager",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/anti_windup_integrator",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/longitudinal_postprocess:lon_postprocess",
        "//onboard/control/math:mpc",
        "//onboard/control/mrac_control",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:interpolation_2d",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/filters:digital_filter",
        "//onboard/math/filters:digital_filter_coefficients",
        "//onboard/math/filters:mean_filter",
        "//onboard/math/geometry:util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "tob_tspkmpc_controller",
    srcs = [
        "tob_tspkmpc_controller.cc",
    ],
    hdrs = [
        "tob_tspkmpc_controller.h",
    ],
    deps = [
        ":controller_base",
        ":controller_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/ap_common/configs/proto:vehicle_config_cc_proto",
        "//onboard/control:control_defs",
        "//onboard/control:control_flags",
        "//onboard/control:control_history_state_manager",
        "//onboard/control:vehicle_state_interface",
        "//onboard/control/anti_windup_integrator",
        "//onboard/control/controllers:predict_vehicle_pose",
        "//onboard/control/longitudinal_postprocess:lon_postprocess",
        "//onboard/control/math:mpc",
        "//onboard/control/mrac_control",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:interpolation_2d",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/filters:digital_filter",
        "//onboard/math/filters:digital_filter_coefficients",
        "//onboard/math/filters:mean_filter",
        "//onboard/math/geometry:util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "control_tracking_statistics",
    srcs = [
        "control_tracking_statistics.cc",
    ],
    hdrs = [
        "control_tracking_statistics.h",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:control_cmd_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "controller_util",
    srcs = ["controller_util.cc"],
    hdrs = [
        "controller_util.h",
    ],
    deps = [
        "//onboard/math:util",
        "//onboard/proto:vehicle_cc_proto",
        "@boost//:circular_buffer",
    ],
)

cc_test(
    name = "control_tracking_statistics_test",
    srcs = ["control_tracking_statistics_test.cc"],
    deps = [
        ":control_tracking_statistics",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "controller_util_test",
    srcs = ["controller_util_test.cc"],
    deps = [
        ":controller_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "predict_vehicle_pose",
    srcs = ["predict_vehicle_pose.cc"],
    hdrs = [
        "predict_vehicle_pose.h",
    ],
    deps = [
        "//onboard/control:control_defs",
        "//onboard/control:control_history_state_manager",
        "//onboard/lite:logging",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cc_library(
    name = "predict_vehicle_pose_test",
    testonly = True,
    srcs = ["predict_vehicle_pose_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        "//onboard/control/controllers:predict_vehicle_pose",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "steer_calibration",
    srcs = ["steer_calibration.cc"],
    hdrs = [
        "steer_calibration.h",
    ],
    deps = [
        ":controller_util",
        "//onboard/control:control_defs",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/proto:control_cmd_cc_proto",
    ],
)

cpplint()
