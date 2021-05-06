load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "selector",
    srcs = ["selector.cc"],
    hdrs = ["selector.h"],
    deps = [
        ":candidate_stats",
        ":selector_input",
        ":traj_cost_features",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/nets/trt:prophnet",
        "//onboard/planner:est_planner_output",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:planner_status",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/selector/proto:selector_debug_cc_proto",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "selector_input",
    hdrs = ["selector_input.h"],
    deps = [
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:route_sections_info",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "cost_feature_base",
    hdrs = ["cost_feature_base.h"],
    deps = [
        "//onboard/planner:est_planner_output",
        "//onboard/planner/scheduler:scheduler_output",
    ],
)

cc_library(
    name = "traj_cost_features",
    srcs = ["traj_cost_features.cc"],
    hdrs = ["traj_cost_features.h"],
    deps = [
        ":candidate_stats",
        ":cost_feature_base",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_sections_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "candidate_stats",
    srcs = ["candidate_stats.cc"],
    hdrs = ["candidate_stats.h"],
    deps = [
        "//onboard/planner:est_planner_output",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/common:planner_status",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/scheduler:scheduler_output",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "selector_test",
    srcs = ["selector_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":selector",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner:est_planner",
        "//onboard/planner:planner_defs",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/common:plot_util",
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
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
