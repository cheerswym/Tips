load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Protos

# Libraries.

cc_library(
    name = "routing_module",
    srcs = ["routing_module.cc"],
    hdrs = ["routing_module.h"],
    data = [
        "//onboard/planner/params:param_files",
        "//onboard/planner/router/params:param_files",
    ],
    deps = [
        ":route",
        ":route_engine",
        ":route_manager",
        ":route_util",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_params",
        "//onboard/planner:teleop_state",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_recorder_module",
    srcs = [],
    hdrs = ["route_recorder_module.h"],
    deps = [
        "//onboard/async:thread_pool",
        "//onboard/lite:lite_module",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_searcher",
    srcs = ["route_searcher.cc"],
    hdrs = ["route_searcher.h"],
    deps = [
        ":route_section_sequence",
        ":route_util",
        ":router_flags",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:coordinate_converter",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/types:span",
    ],
)

filegroup(
    name = "route_data",
    data = glob([
        "testdata/*.pb.txt",
        "testdata/routes/**",
    ]),
)

cc_library(
    name = "route",
    srcs = ["route.cc"],
    hdrs = ["route.h"],
    data = [
        ":route_data",
    ],
    deps = [
        ":route_section_sequence",
        ":route_util",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/planner:composite_lane_path",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "route_util",
    srcs = ["route_util.cc"],
    hdrs = ["route_util.h"],
    deps = [
        ":drive_passage",
        ":route_sections_info",
        ":router_flags",
        "//common/proto:map_geometry_cc_proto",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:coordinate_converter",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_params",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:proto_util",
        "//onboard/utils:time_util",
        "@boost//:geometry",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":drive_passage",
        ":route_sections",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:util",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
    ],
)

cc_library(
    name = "route_manager_output",
    srcs = ["route_manager_output.cc"],
    hdrs = ["route_manager_output.h"],
    deps = [
        ":route",
        "//onboard/global:trace",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_manager",
    srcs = ["route_manager.cc"],
    hdrs = ["route_manager.h"],
    deps = [
        ":multi_stops_request",
        ":off_road_route_searcher",
        ":route",
        ":route_ego_tracker",
        ":route_error",
        ":route_manager_output",
        ":route_manager_state",
        ":route_manager_util",
        ":route_reroute",
        ":route_util",
        ":router_flags",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/eval:qevent",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:safe_unit",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_flags",
        "//onboard/planner:teleop_state",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf_headers",
    ],
)

cc_library(
    name = "route_section_sequence",
    srcs = ["route_section_sequence.cc"],
    hdrs = ["route_section_sequence.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "router_flags",
    srcs = ["router_flags.cc"],
    hdrs = ["router_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "drive_passage",
    srcs = ["drive_passage.cc"],
    hdrs = ["drive_passage.h"],
    deps = [
        "//onboard/container:strong_vector",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "route_lane_path_finder",
    srcs = ["route_lane_path_finder.cc"],
    hdrs = ["route_lane_path_finder.h"],
    deps = [
        ":plot_util",
        ":route_section_sequence",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "drive_passage_builder",
    srcs = ["drive_passage_builder.cc"],
    hdrs = ["drive_passage_builder.h"],
    deps = [
        ":drive_passage",
        ":route_util",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_util",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner:planner_defs",
    ],
)

cc_library(
    name = "route_test_util",
    testonly = True,
    srcs = ["route_test_util.cc"],
    hdrs = ["route_test_util.h"],
    deps = [
        ":plot_util",
        ":route",
        ":route_engine",
        ":route_sections",
        ":route_sections_util",
        ":router_flags",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "route_engine",
    srcs = ["route_engine.cc"],
    hdrs = ["route_engine.h"],
    deps = [
        ":route_lane_path_finder",
        ":route_searcher",
        ":route_section_sequence",
        ":route_util",
        ":router_flags",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "multi_stops_request",
    srcs = ["multi_stops_request.cc"],
    hdrs = ["multi_stops_request.h"],
    deps = [
        ":route",
        ":route_util",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_manager_util",
    srcs = ["route_manager_util.cc"],
    hdrs = ["route_manager_util.h"],
    deps = [
        ":multi_stops_request",
        ":route_engine",
        ":route_lane_path_finder",
        ":router_flags",
        "//onboard/global:clock",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "route_sections",
    srcs = ["route_sections.cc"],
    hdrs = ["route_sections.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "route_sections_info",
    srcs = ["route_sections_info.cc"],
    hdrs = ["route_sections_info.h"],
    deps = [
        ":route_sections",
        "//onboard/global:logging",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_semantic_map_manager",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "route_sections_util",
    srcs = ["route_sections_util.cc"],
    hdrs = ["route_sections_util.h"],
    deps = [
        ":drive_passage_builder",
        ":plot_util",
        ":route_section_sequence",
        ":route_sections",
        ":route_sections_info",
        ":route_util",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_frame",
        "//onboard/planner:planner_defs",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "map_match",
    srcs = ["map_match.cc"],
    hdrs = ["map_match.h"],
    deps = [
        ":route_util",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_error",
    srcs = ["route_error.cc"],
    hdrs = ["route_error.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "route_reroute",
    srcs = ["route_reroute.cc"],
    hdrs = ["route_reroute.h"],
    deps = [
        "multi_stops_request",
        ":map_match",
        ":route",
        ":route_engine",
        ":route_error",
        ":route_manager_util",
        ":route_util",
        ":router_flags",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "off_road_route_searcher",
    srcs = ["off_road_route_searcher.cc"],
    hdrs = ["off_road_route_searcher.h"],
    deps = [
        ":multi_stops_request",
        ":route_manager_util",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/math/geometry:line_intersection",
        "//onboard/planner:composite_lane_path",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "route_portal_util",
    srcs = ["route_portal_util.cc"],
    hdrs = ["route_portal_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/proto:q_command_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_protobuf//:protobuf_headers",
    ],
)

cc_library(
    name = "route_manager_state",
    hdrs = ["route_manager_state.h"],
    deps = [
        ":multi_stops_request",
        "//onboard/proto:route_cc_proto",
    ],
)

cc_library(
    name = "route_ego_tracker",
    srcs = ["route_ego_tracker.cc"],
    hdrs = ["route_ego_tracker.h"],
    deps = [
        ":route_manager_state",
        ":route_util",
        "//onboard/global:clock",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:composite_lane_path",
        "//onboard/proto:route_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "route_validation_test",
    srcs = ["route_validation_test.cc"],
    data = [
        ":route_data",
    ],
    deps = [
        ":route",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_util_test",
    size = "large",
    srcs = ["route_util_test.cc"],
    deps = [
        ":route_searcher",
        ":route_util",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_searcher_test",
    srcs = ["route_searcher_test.cc"],
    deps = [
        ":route",
        ":route_searcher",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "drive_passage_test",
    srcs = ["drive_passage_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":drive_passage",
        ":drive_passage_builder",
        ":plot_util",
        ":route_sections_util",
        ":route_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/planner:planner_flags",
        "//onboard/planner/test_util:route_builder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_test",
    srcs = ["route_test.cc"],
    deps = [
        ":route",
        ":route_engine",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/planner/test_util:route_builder",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/utils:test_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_manager_test",
    srcs = ["route_manager_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":route_ego_tracker",
        ":route_manager",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

# cc_test(
#     name = "section_sequence_test",
#     srcs = ["section_sequence_test.cc"],
#     deps = [
#         ":route",
#         ":route_searcher",
#         "//onboard/global:test_main",
#         "//onboard/utils:test_util",
#         "@com_github_google_glog//:glog",
#     ],
# )

cc_test(
    name = "route_lane_path_finder_test",
    srcs = ["route_lane_path_finder_test.cc"],
    deps = [
        ":plot_util",
        ":route_lane_path_finder",
        ":route_searcher",
        "//onboard/global:test_main",
        "//onboard/global:timer",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "drive_passage_builder_test",
    testonly = True,
    srcs = ["drive_passage_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":drive_passage_builder",
        ":plot_util",
        ":route_sections_info",
        ":route_test_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner:planner_defs",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_flags_convert_test",
    srcs = ["route_flags_convert_test.cc"],
    deps = [
        ":route_util",
        ":router_flags",
        "//onboard/global:test_main",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_engine_test",
    srcs = ["route_engine_test.cc"],
    deps = [
        ":plot_util",
        ":route_engine",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "multi_stops_request_test",
    srcs = ["multi_stops_request_test.cc"],
    deps = [
        ":multi_stops_request",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_manager_util_test",
    srcs = ["route_manager_util_test.cc"],
    deps = [
        ":route_manager_util",
        "//onboard/global:test_main",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_sections_info_test",
    srcs = ["route_sections_info_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":plot_util",
        ":route_sections_info",
        ":route_test_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_sections_util_test",
    srcs = ["route_sections_util_test.cc"],
    deps = [
        ":route_sections_util",
        "//onboard/global:test_main",
        "//onboard/planner:planner_semantic_map_manager",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "off_road_route_searcher_test",
    srcs = ["off_road_route_searcher_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":off_road_route_searcher",
        ":plot_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/common:plot_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_params_test",
    srcs = ["route_params_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "route_portal_util_test",
    srcs = ["route_portal_util_test.cc"],
    deps = [
        ":route_portal_util",
        "//onboard/global:test_main",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
