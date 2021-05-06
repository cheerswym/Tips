load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "speed_finder",
    srcs = ["speed_finder.cc"],
    hdrs = ["speed_finder.h"],
    deps = [
        ":constraint_generator",
        ":constraint_manager_decider",
        ":freespace_speed_finder_input",
        ":freespace_speed_finder_output",
        ":ignore_decider",
        ":interactive_speed_decision",
        ":path_semantic_analyzer",
        ":path_speed_combiner",
        ":plot_util",
        ":speed_finder_input",
        ":speed_finder_output",
        ":speed_limit",
        ":speed_limit_generator",
        ":speed_optimizer",
        ":speed_vector",
        ":st_graph",
        ":st_overlap_analyzer",
        ":standstill_distance_decider",
        ":time_buffer_decider",
        "//onboard/async:async_macro",
        "//onboard/async:async_util",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:maps_helper",
        "//onboard/math:vec",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:multi_timer_util",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/speed/decider:post_st_boundary_modifier",
        "//onboard/planner/speed/decider:pre_st_boundary_modifier",
        "//onboard/planner/speed/decider:st_boundary_pre_decider",
        "//onboard/planner/util:path_util",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "speed_finder_input",
    hdrs = ["speed_finder_input.h"],
    deps = [
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_sections",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "speed_finder_output",
    hdrs = ["speed_finder_output.h"],
    deps = [
        "//onboard/planner:trajectory_end_info",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "freespace_speed_finder_input",
    hdrs = ["freespace_speed_finder_input.h"],
    deps = [
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:discretized_path",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "freespace_speed_finder_output",
    hdrs = ["freespace_speed_finder_output.h"],
    deps = [
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "st_point",
    hdrs = ["st_point.h"],
    deps = [
        "//onboard/math:vec",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "vt_point",
    hdrs = ["vt_point.h"],
    deps = [
        "//onboard/math:vec",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "speed_point",
    hdrs = ["speed_point.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "speed_limit",
    srcs = ["speed_limit.cc"],
    hdrs = ["speed_limit.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "st_boundary",
    srcs = ["st_boundary.cc"],
    hdrs = ["st_boundary.h"],
    deps = [
        ":vt_point",
        "//onboard/lite:logging",
        "//onboard/math/geometry:common_shapes",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/speed:st_point",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/prediction:predicted_trajectory",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "st_boundary_with_decision",
    srcs = ["st_boundary_with_decision.cc"],
    hdrs = ["st_boundary_with_decision.h"],
    deps = [
        ":st_boundary",
        ":vt_point",
        "//onboard/lite:logging",
        "//onboard/planner/speed:st_point",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "speed_vector",
    srcs = ["speed_vector.cc"],
    hdrs = ["speed_vector.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
    ],
)

cc_library(
    name = "st_graph",
    srcs = ["st_graph.cc"],
    hdrs = ["st_graph.h"],
    deps = [
        ":st_boundary",
        ":st_boundary_with_decision",
        ":st_point",
        ":vt_point",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:parallel_for",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/maps:maps_helper",
        "//onboard/math/geometry:common_shapes",
        "//onboard/math/geometry:kdtree",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_params",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = ["speed_optimizer.cc"],
    hdrs = ["speed_optimizer.h"],
    deps = [
        ":speed_limit",
        ":speed_limit_generator",
        ":speed_optimizer_constraint_manager",
        ":speed_vector",
        ":st_boundary",
        ":st_boundary_with_decision",
        ":st_point",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_params",
        "//onboard/planner/math/piecewise_jerk_qp_solver",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "speed_limit_generator",
    srcs = ["speed_limit_generator.cc"],
    hdrs = ["speed_limit_generator.h"],
    deps = [
        ":speed_limit",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:maps_helper",
        "//onboard/math:coordinate_converter",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/planner:discretized_path",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:path_util",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "path_speed_combiner",
    srcs = ["path_speed_combiner.cc"],
    hdrs = ["path_speed_combiner.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/speed:speed_vector",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "path_semantic_analyzer",
    srcs = ["path_semantic_analyzer.cc"],
    hdrs = ["path_semantic_analyzer.h"],
    deps = [
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:util",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/util:path_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":speed_vector",
        ":st_boundary",
        ":st_boundary_with_decision",
        "//offboard/vis/vantage/charts:chart_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:trace",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_util",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/vis/canvas/proto:canvas_buffer_cc_proto",
        "//onboard/vis/common:color",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "st_overlap_analyzer",
    srcs = ["st_overlap_analyzer.cc"],
    hdrs = ["st_overlap_analyzer.h"],
    deps = [
        ":path_semantic_analyzer",
        ":st_boundary",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/maps:lane_point",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/planner/util:path_util",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "ignore_decider",
    srcs = ["ignore_decider.cc"],
    hdrs = ["ignore_decider.h"],
    deps = [
        ":path_semantic_analyzer",
        ":st_boundary_with_decision",
        ":st_point",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/planner:discretized_path",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/util:path_util",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "constraint_generator",
    srcs = ["constraint_generator.cc"],
    hdrs = ["constraint_generator.h"],
    deps = [
        ":st_boundary_with_decision",
        ":st_graph",
        "//onboard/lite:logging",
        "//onboard/planner:discretized_path",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/speed/decider:close_object_slowdown_decider",
    ],
)

cc_library(
    name = "constraint_manager_decider",
    srcs = ["constraint_manager_decider.cc"],
    hdrs = ["constraint_manager_decider.h"],
    deps = [
        ":st_boundary_with_decision",
        "//onboard/lite:logging",
        "//onboard/planner/decision:constraint_manager",
    ],
)

cc_library(
    name = "standstill_distance_decider",
    srcs = ["standstill_distance_decider.cc"],
    hdrs = ["standstill_distance_decider.h"],
    deps = [
        ":st_boundary_with_decision",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "time_buffer_decider",
    srcs = ["time_buffer_decider.cc"],
    hdrs = ["time_buffer_decider.h"],
    deps = [
        ":st_boundary_with_decision",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
    ],
)

cc_library(
    name = "st_graph_data",
    hdrs = ["st_graph_data.h"],
    deps = [
        ":speed_limit",
        ":st_boundary",
        ":st_boundary_with_decision",
    ],
)

# Dp-sampling
cc_library(
    name = "svt_point",
    hdrs = ["svt_point.h"],
    deps = [
        ":st_point",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "svt_graph_point",
    hdrs = ["svt_graph_point.h"],
    deps = [
        ":svt_point",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "dp_svt_cost",
    srcs = ["dp_svt_cost.cc"],
    hdrs = ["dp_svt_cost.h"],
    deps = [
        ":st_boundary",
        ":st_boundary_with_decision",
        ":svt_graph_point",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/proto:planner_params_cc_proto",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "gridded_svt_graph",
    srcs = ["gridded_svt_graph.cc"],
    hdrs = ["gridded_svt_graph.h"],
    deps = [
        ":dp_svt_cost",
        ":speed_limit",
        ":speed_vector",
        ":st_graph_data",
        ":svt_graph_point",
        "//onboard/async:parallel_for",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "empty_road_speed",
    srcs = ["empty_road_speed.cc"],
    hdrs = ["empty_road_speed.h"],
    deps = [
        ":speed_limit",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:speed_profile",
        "//onboard/planner/proto:planner_params_cc_proto",
    ],
)

cc_library(
    name = "interactive_speed_decision",
    srcs = ["interactive_speed_decision.cc"],
    hdrs = ["interactive_speed_decision.h"],
    deps = [
        ":empty_road_speed",
        ":gridded_svt_graph",
        ":speed_limit",
        ":speed_point",
        ":speed_vector",
        ":st_boundary",
        ":st_boundary_with_decision",
        ":st_graph",
        ":st_graph_data",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:buffered_logger",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:fast_math",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math/geometry:halfplane",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner:speed_profile",
        "//onboard/planner:trajectory_point",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/math:intelligent_driver_model",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/object:spacetime_object_state",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/speed/decider:post_st_boundary_modifier",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:status_macros",
        "//onboard/utils:terminal_color",
        "//onboard/vis/common:color",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "speed_optimizer_constraint_manager",
    srcs = ["speed_optimizer_constraint_manager.cc"],
    hdrs = ["speed_optimizer_constraint_manager.h"],
    deps = [
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "st_close_trajectory",
    srcs = ["st_close_trajectory.cc"],
    hdrs = ["st_close_trajectory.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "speed_optimizer_test",
    srcs = ["speed_optimizer_test.cc"],
    deps = [
        ":speed_optimizer",
        ":st_point",
        ":vt_point",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/params:param_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_speed_combiner_test",
    srcs = ["path_speed_combiner_test.cc"],
    deps = [
        ":path_speed_combiner",
        "//onboard/global:test_main",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_limit_test",
    srcs = ["speed_limit_test.cc"],
    deps = [
        ":speed_limit",
        "//onboard/global:test_main",
        "@com_google_absl//absl/status",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
