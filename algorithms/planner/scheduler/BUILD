load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "scheduler_input",
    hdrs = ["scheduler_input.h"],
    deps = [
        ":smooth_reference_line_result",
        "//onboard/planner:planner_params",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:route",
        "//onboard/planner/router:route_sections_info",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "scheduler_output",
    hdrs = ["scheduler_output.h"],
    deps = [
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
    ],
)

cc_library(
    name = "target_lane_clearance",
    srcs = ["target_lane_clearance.cc"],
    hdrs = ["target_lane_clearance.h"],
    deps = [
        ":scheduler_output",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_defs",
        "//onboard/planner/math:rss_formulas",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:route_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "scheduler_util",
    srcs = ["scheduler_util.cc"],
    hdrs = ["scheduler_util.h"],
    deps = [
        ":scheduler_output",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler/proto:scheduler_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "path_boundary_builder",
    srcs = ["path_boundary_builder.cc"],
    hdrs = ["path_boundary_builder.h"],
    deps = [
        ":path_boundary_builder_helper",
        ":smooth_reference_line_result",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "path_boundary_builder_helper",
    srcs = ["path_boundary_builder_helper.cc"],
    hdrs = ["path_boundary_builder_helper.h"],
    deps = [
        ":scheduler_plot_util",
        ":smooth_reference_line_result",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "local_map_builder",
    srcs = ["local_map_builder.cc"],
    hdrs = ["local_map_builder.h"],
    deps = [
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_info",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "scheduler_plot_util",
    srcs = ["scheduler_plot_util.cc"],
    hdrs = ["scheduler_plot_util.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/scheduler/lane_graph",
    ],
)

cc_library(
    name = "target_lane_path_filter",
    srcs = ["target_lane_path_filter.cc"],
    hdrs = ["target_lane_path_filter.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/router:route_sections_info",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "multi_tasks_scheduler",
    srcs = ["multi_tasks_scheduler.cc"],
    hdrs = ["multi_tasks_scheduler.h"],
    deps = [
        ":path_boundary_builder",
        ":scheduler_input",
        ":scheduler_output",
        ":scheduler_util",
        ":target_lane_clearance",
        "//onboard/async:parallel_for",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "reference_line_qp_smoother",
    srcs = ["reference_line_qp_smoother.cc"],
    hdrs = ["reference_line_qp_smoother.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/qp:osqp_solver",
        "//onboard/planner/scheduler/proto:reference_line_qp_smoother_cc_proto",
    ],
)

cc_library(
    name = "smooth_reference_line_builder",
    srcs = ["smooth_reference_line_builder.cc"],
    hdrs = ["smooth_reference_line_builder.h"],
    deps = [
        ":path_boundary_builder",
        ":path_boundary_builder_helper",
        ":reference_line_qp_smoother",
        ":smooth_reference_line_result",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
    ],
)

cc_library(
    name = "smooth_reference_line_result",
    srcs = ["smooth_reference_line_result.cc"],
    hdrs = ["smooth_reference_line_result.h"],
    deps = [
        "//onboard/maps:lane_point",
        "//onboard/math:piecewise_linear_function",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "target_lane_clearance_test",
    srcs = ["target_lane_clearance_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":target_lane_clearance",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/planner:planner_params",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_boundary_builder_test",
    srcs = ["path_boundary_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":path_boundary_builder",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "target_lane_path_filter_test",
    srcs = ["target_lane_path_filter_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":scheduler_plot_util",
        ":target_lane_path_filter",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/scheduler/lane_graph:lane_graph_builder",
        "//onboard/planner/scheduler/lane_graph:lane_path_finder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "reference_line_qp_smoother_test",
    srcs = ["reference_line_qp_smoother_test.cc"],
    deps = [
        ":reference_line_qp_smoother",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "smooth_reference_line_builder_test",
    srcs = ["smooth_reference_line_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":path_boundary_builder",
        ":smooth_reference_line_builder",
        ":smooth_reference_line_result",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "smooth_reference_line_result_test",
    srcs = ["smooth_reference_line_result_test.cc"],
    deps = [
        ":smooth_reference_line_builder",
        ":smooth_reference_line_result",
        "//onboard/global:test_main",
        "//onboard/planner:planner_semantic_map_manager",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
