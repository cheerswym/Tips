load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "initializer_input",
    hdrs = ["initializer_input.h"],
    deps = [
        ":collision_checker",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/initializer/geometry:geometry_form_builder",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "initializer_output",
    hdrs = ["initializer_output.h"],
    deps = [
        ":cost_provider",
        ":motion_graph",
        ":motion_state",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
    ],
)

cc_library(
    name = "motion_state",
    hdrs = ["motion_state.h"],
    deps = [
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "motion_form",
    srcs = ["motion_form.cc"],
    hdrs = ["motion_form.h"],
    deps = [
        ":motion_state",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/initializer/geometry:geometry_form",
    ],
)

cc_library(
    name = "expert_complete_motion_form",
    srcs = ["expert_complete_motion_form.cc"],
    hdrs = ["expert_complete_motion_form.h"],
    deps = [
        ":motion_form",
        ":motion_state",
        "//offboard/planner/ml/datasets/pnc_scenario_dataset:proto_converter",
        "//offboard/planner/ml/datasets/pnc_scenario_dataset/proto:pnc_scenario_cc_proto",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/initializer/geometry:geometry_form",
        "//onboard/planner/initializer/geometry:geometry_form_builder",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:history_buffer",
    ],
)

cc_library(
    name = "candidate_complete_motion_form",
    srcs = ["candidate_complete_motion_form.cc"],
    hdrs = ["candidate_complete_motion_form.h"],
    deps = [
        ":motion_form",
        ":motion_search_util",
        ":motion_state",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/initializer/geometry:geometry_form",
        "//onboard/planner/initializer/geometry:geometry_form_builder",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
    ],
)

cc_library(
    name = "motion_graph",
    srcs = ["motion_graph.cc"],
    hdrs = ["motion_graph.h"],
    deps = [
        ":motion_form",
        ":motion_state",
        "//onboard/container:strong_vector",
        "//onboard/global:logging",
        "//onboard/math:vec",
        "//onboard/planner:planner_defs",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "cost_feature",
    hdrs = ["cost_feature.h"],
    deps = [
        ":motion_graph",
        ":ref_speed_table",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/proto:initializer_config_cc_proto",
    ],
)

cc_library(
    name = "dp_cost_feature",
    srcs = ["dp_cost_feature.cc"],
    hdrs = ["dp_cost_feature.h"],
    deps = [
        ":collision_checker",
        ":cost_feature",
        ":motion_graph",
        ":ref_speed_table",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/initializer/proto:initializer_config_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/proto:planner_params_cc_proto",
    ],
)

cc_library(
    name = "reference_line_cost_feature",
    srcs = ["reference_line_cost_feature.cc"],
    hdrs = ["reference_line_cost_feature.h"],
    deps = [
        ":collision_checker",
        ":cost_feature",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "reference_line_searcher",
    srcs = ["reference_line_searcher.cc"],
    hdrs = ["reference_line_searcher.h"],
    deps = [
        ":cost_provider",
        ":initializer_input",
        ":initializer_output",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:vehicle_geometry_util",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "cost_provider",
    srcs = ["cost_provider.cc"],
    hdrs = ["cost_provider.h"],
    deps = [
        ":brute_force_collision_checker",
        ":collision_checker",
        ":cost_feature",
        ":dp_cost_feature",
        ":motion_graph",
        ":ref_speed_table",
        ":reference_line_cost_feature",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/initializer/proto:initializer_config_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/prediction:prediction_defs",
    ],
)

cc_library(
    name = "search_motion",
    srcs = ["search_motion.cc"],
    hdrs = ["search_motion.h"],
    deps = [
        ":brute_force_collision_checker",
        ":dp_motion_searcher",
        ":initializer_input",
        ":initializer_output",
        ":initializer_util",
        ":reference_line_searcher",
        "//onboard/async:async_macro",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_params",
        "//onboard/planner/common:multi_timer_util",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/decision:leading_object",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/geometry:geometry_graph_builder",
        "//onboard/planner/initializer/geometry:geometry_graph_cache",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/initializer/proto:initializer_config_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "multi_traj_selector",
    srcs = ["multi_traj_selector.cc"],
    hdrs = ["multi_traj_selector.h"],
    deps = [
        ":cost_provider",
        ":initializer_output",
        ":motion_graph",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/math/geometry:box2d",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/math:rss_formulas",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/scheduler:target_lane_clearance",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "initializer_util",
    srcs = ["initializer_util.cc"],
    hdrs = ["initializer_util.h"],
    deps = [
        ":initializer_output",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/proto:charts_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/vis/common:colormap",
    ],
)

cc_library(
    name = "dp_motion_searcher",
    srcs = ["dp_motion_searcher.cc"],
    hdrs = ["dp_motion_searcher.h"],
    deps = [
        ":candidate_complete_motion_form",
        ":cost_provider",
        ":expert_complete_motion_form",
        ":initializer_input",
        ":initializer_output",
        ":motion_graph",
        ":motion_search_util",
        ":motion_state",
        ":multi_traj_selector",
        "//offboard/planner/ml/datasets/pnc_scenario_dataset:proto_converter",
        "//offboard/planner/ml/datasets/pnc_scenario_dataset/proto:pnc_scenario_cc_proto",
        "//onboard/async:async_macro",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:small_set",
        "//onboard/eval:qevent",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/initializer:motion_graph_cache",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:route_util",
        "//onboard/utils:file_util",
        "//onboard/utils:history_buffer",
        "@com_google_absl//absl/algorithm:container",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "motion_graph_cache",
    srcs = ["motion_graph_cache.cc"],
    hdrs = ["motion_graph_cache.h"],
    deps = [
        "motion_form",
        "motion_graph",
        "//onboard/async:async_macro",
        "//onboard/async:async_util",
        "//onboard/planner/initializer/geometry:geometry_form",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "motion_search_util",
    srcs = ["motion_search_util.cc"],
    hdrs = ["motion_search_util.h"],
    deps = [
        ":motion_graph",
        ":motion_state",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "collision_checker",
    srcs = ["collision_checker.cc"],
    hdrs = ["collision_checker.h"],
    deps = [
        ":motion_form",
        "//onboard/math/geometry:aabox2d",
        "//onboard/planner/initializer:cost_feature",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "brute_force_collision_checker",
    srcs = ["brute_force_collision_checker.cc"],
    hdrs = ["brute_force_collision_checker.h"],
    deps = [
        ":collision_checker",
        ":motion_state",
        "//onboard/planner/initializer/geometry:geometry_state",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "ref_speed_table",
    srcs = ["ref_speed_table.cc"],
    hdrs = ["ref_speed_table.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/planner:planner_defs",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "test_util",
    testonly = True,
    srcs = ["test_util.cc"],
    hdrs = ["test_util.h"],
    deps = [
        ":motion_graph",
        ":motion_state",
        "//onboard/math:vec",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/types:span",
        "@com_google_googletest//:gtest",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "search_motion_test",
    srcs = ["search_motion_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":dp_motion_searcher",
        ":initializer_util",
        ":multi_traj_selector",
        ":search_motion",
        ":test_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/planner:planner_defs",
        "//onboard/planner/initializer/geometry:geometry_graph",
        "//onboard/planner/initializer/geometry:geometry_graph_builder",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:trajectory_plot_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "motion_form_test",
    srcs = ["motion_form_test.cc"],
    deps = [
        ":motion_form",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/planner/initializer/geometry:geometry_form",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "collision_checker_test",
    srcs = ["collision_checker_test.cc"],
    deps = [
        ":collision_checker",
        ":cost_feature",
        ":test_util",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/planner/initializer:brute_force_collision_checker",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:source_location",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "dp_cost_feature_test",
    srcs = ["dp_cost_feature_test.cc"],
    deps = [
        ":dp_cost_feature",
        "//onboard/global:test_main",
        "//onboard/planner:planner_defs",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "ref_speed_table_test",
    srcs = ["ref_speed_table_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":ref_speed_table",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_flags",
        "//onboard/planner/initializer:initializer_util",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "test_util_test",
    srcs = ["test_util_test.cc"],
    deps = [
        ":motion_graph",
        ":test_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "reference_line_searcher_test",
    srcs = ["reference_line_searcher_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":collision_checker",
        ":initializer_input",
        ":initializer_output",
        ":initializer_util",
        ":motion_graph",
        ":reference_line_searcher",
        ":search_motion",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/initializer/geometry:geometry_form_builder",
        "//onboard/planner/initializer/geometry:geometry_graph_builder",
        "//onboard/planner/initializer/geometry:geometry_graph_cache",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:trajectory_plot_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "multi_traj_selector_test",
    srcs = ["multi_traj_selector_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":collision_checker",
        ":initializer_input",
        ":initializer_output",
        ":initializer_util",
        ":motion_graph",
        ":multi_traj_selector",
        ":search_motion",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "//onboard/planner/initializer/geometry:geometry_form_builder",
        "//onboard/planner/initializer/geometry:geometry_graph_builder",
        "//onboard/planner/initializer/geometry:geometry_graph_cache",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/scheduler:scheduler_plot_util",
        "//onboard/planner/scheduler:scheduler_util",
        "//onboard/planner/scheduler/lane_graph:lane_graph_builder",
        "//onboard/planner/scheduler/lane_graph:lane_path_finder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:trajectory_plot_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
