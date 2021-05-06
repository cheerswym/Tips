load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "ddp_optimizer",
    srcs = ["ddp_optimizer.cc"],
    hdrs = ["ddp_optimizer.h"],
    deps = [
        ":ddp_cost_manager_hook",
        ":ddp_optimizer_hook",
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning:auto_tuning_common_flags",
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning/proto:auto_tuning_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/eval:qevent",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "//onboard/planner/proto:planner_params_cc_proto",
    ],
)

cc_library(
    name = "ddp_optimizer_hook",
    hdrs = ["ddp_optimizer_hook.h"],
    deps = [
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "ddp_optimizer_debug_hook",
    hdrs = ["ddp_optimizer_debug_hook.h"],
    deps = [
        ":ddp_optimizer_hook",
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/util:trajectory_plot_util",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "ddp_cost_manager_hook",
    hdrs = ["ddp_cost_manager_hook.h"],
    deps = [
        ":ddp_optimizer_hook",
        "//onboard/math:eigen",
        "//onboard/planner/optimization/problem:cost",
        "//onboard/planner/optimization/problem:cost_helper",
    ],
)

cc_library(
    name = "trajectory_optimizer_defs",
    hdrs = ["trajectory_optimizer_defs.h"],
    deps = [
        "//onboard/planner:planner_defs",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
    ],
)

cc_library(
    name = "object_cost_util",
    srcs = ["object_cost_util.cc"],
    hdrs = ["object_cost_util.h"],
    deps = [
        ":trajectory_optimizer_defs",
        "//offboard/vis/vantage/charts:chart_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_macro",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/planner:planner_params",
        "//onboard/planner:planner_util",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/optimization/problem:aggregate_static_object_cost",
        "//onboard/planner/optimization/problem:av_model_helper",
        "//onboard/planner/optimization/problem:mfob_emeraude_object_cost",
        "//onboard/planner/optimization/problem:partitioned_object_cost",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_optimizer_util",
    srcs = ["trajectory_optimizer_util.cc"],
    hdrs = ["trajectory_optimizer_util.h"],
    deps = [
        ":trajectory_optimizer_defs",
        "//offboard/vis/vantage/charts:chart_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/planner:planner_params",
        "//onboard/planner:planner_util",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/optimization/ipopt:ipopt_adapter",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "//onboard/planner/util:trajectory_plot_util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:btree",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_optimizer",
    srcs = ["trajectory_optimizer.cc"],
    hdrs = ["trajectory_optimizer.h"],
    data = [
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning:dopt_auto_tuning_files",
    ],
    deps = [
        ":ddp_optimizer",
        ":ddp_optimizer_debug_hook",
        ":object_cost_util",
        ":trajectory_optimizer_defs",
        ":trajectory_optimizer_input",
        ":trajectory_optimizer_output",
        ":trajectory_optimizer_util",
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning:auto_tuning_utils",
        "//offboard/vis/vantage/charts:chart_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_macro",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/planner:mfob_trajectory_smoother",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_params",
        "//onboard/planner:planner_util",
        "//onboard/planner:trajectory_point",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/optimization/problem:aggregate_static_object_cost",
        "//onboard/planner/optimization/problem:av_model_helper",
        "//onboard/planner/optimization/problem:center_line_query_helper",
        "//onboard/planner/optimization/problem:curvature_cost",
        "//onboard/planner/optimization/problem:curvature_deviation_cost",
        "//onboard/planner/optimization/problem:end_heading_cost",
        "//onboard/planner/optimization/problem:forward_speed_cost",
        "//onboard/planner/optimization/problem:intrinsic_jerk_cost",
        "//onboard/planner/optimization/problem:lateral_acceleration_cost",
        "//onboard/planner/optimization/problem:longitudinal_acceleration_cost",
        "//onboard/planner/optimization/problem:mfob_curvature_rate_cost",
        "//onboard/planner/optimization/problem:mfob_curvature_rate_rate_cost",
        "//onboard/planner/optimization/problem:mfob_emeraude_object_cost",
        "//onboard/planner/optimization/problem:mfob_gradient_fixed_projective_object_cost",
        "//onboard/planner/optimization/problem:mfob_intrinsic_lateral_snap_cost",
        "//onboard/planner/optimization/problem:mfob_lateral_jerk_cost",
        "//onboard/planner/optimization/problem:mfob_path_boundary_cost",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "//onboard/planner/optimization/problem:msd_static_boundary_cost",
        "//onboard/planner/optimization/problem:partitioned_object_cost",
        "//onboard/planner/optimization/problem:reference_control_deviation_cost",
        "//onboard/planner/optimization/problem:reference_line_deviation_cost",
        "//onboard/planner/optimization/problem:reference_longitudinal_jerk_deviation_cost",
        "//onboard/planner/optimization/problem:reference_state_deviation_cost",
        "//onboard/planner/optimization/problem:segmented_speed_limit_cost",
        "//onboard/planner/optimization/problem:static_boundary_cost",
        "//onboard/planner/util:trajectory_plot_util",
        "//onboard/proto:charts_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_optimizer_input",
    hdrs = ["trajectory_optimizer_input.h"],
    deps = [
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_optimizer_output",
    hdrs = ["trajectory_optimizer_output.h"],
    deps = [
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning/proto:auto_tuning_cc_proto",
        "//onboard/planner:trajectory_point",
        "//onboard/proto:planner_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "ddp_optimizer_test",
    srcs = ["ddp_optimizer_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":ddp_optimizer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/global:trace",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_defs",
        "//onboard/planner/optimization/problem:curvature_cost",
        "//onboard/planner/optimization/problem:forward_speed_cost",
        "//onboard/planner/optimization/problem:intrinsic_jerk_cost",
        "//onboard/planner/optimization/problem:longitudinal_acceleration_cost",
        "//onboard/planner/optimization/problem:reference_control_deviation_cost",
        "//onboard/planner/optimization/problem:reference_line_deviation_cost",
        "//onboard/planner/optimization/problem:reference_state_deviation_cost",
        "//onboard/planner/optimization/problem:segmented_speed_limit_cost",
        "//onboard/planner/optimization/problem:static_boundary_cost",
        "//onboard/planner/optimization/problem:tob_curvature_rate_cost",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:trajectory_plot_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "trajectory_optimizer_test",
    srcs = ["trajectory_optimizer_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":trajectory_optimizer",
        ":trajectory_optimizer_input",
        ":trajectory_optimizer_output",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_util",
        "//onboard/planner/decision:constraint_builder",
        "//onboard/planner/decision:end_of_path_boundary",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:trajectory_plot_util",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
