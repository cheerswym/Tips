load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "basic_graph",
    hdrs = ["basic_graph.h"],
    deps = [
        "//onboard/lite:logging",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "lane_graph",
    hdrs = ["lane_graph.h"],
    deps = [
        ":basic_graph",
    ],
)

cc_library(
    name = "lane_graph_builder",
    srcs = ["lane_graph_builder.cc"],
    hdrs = ["lane_graph_builder.h"],
    deps = [
        ":lane_graph",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_sections_util",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "dijkstra",
    srcs = ["dijkstra.cc"],
    hdrs = ["dijkstra.h"],
    deps = [
        ":basic_graph",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "lane_path_finder",
    srcs = ["lane_path_finder.cc"],
    hdrs = ["lane_path_finder.h"],
    deps = [
        ":dijkstra",
        ":lane_graph",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/common:lane_path_info",
        "//onboard/planner/router:route_sections_info",
        "//onboard/utils:status_macros",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "basic_graph_test",
    srcs = ["basic_graph_test.cc"],
    deps = [
        ":basic_graph",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "dijkstra_test",
    srcs = ["dijkstra_test.cc"],
    deps = [
        ":dijkstra",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lane_path_finder_test",
    srcs = ["lane_path_finder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lane_graph_builder",
        ":lane_path_finder",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/router:route_test_util",
        "//onboard/planner/scheduler:scheduler_plot_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
