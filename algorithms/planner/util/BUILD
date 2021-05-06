load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Libraries.

cc_library(
    name = "trajectory_plot_util",
    srcs = ["trajectory_plot_util.cc"],
    hdrs = ["trajectory_plot_util.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math:vec",
        "//onboard/planner:planner_defs",
        "//onboard/planner:trajectory_point",
        "//onboard/proto:trajectory_point_cc_proto",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    deps = [
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/planner:planner_defs",
        "//onboard/prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "vehicle_geometry_util",
    srcs = ["vehicle_geometry_util.cc"],
    hdrs = ["vehicle_geometry_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:offset_rect",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "perception_util",
    srcs = ["perception_util.cc"],
    hdrs = ["perception_util.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "path_util",
    hdrs = ["path_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/proto:trajectory_point_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_status_macros",
    hdrs = ["planner_status_macros.h"],
    deps = [
        "//onboard/planner/common:planner_status",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "min_segment_distance_problem",
    srcs = ["min_segment_distance_problem.cc"],
    hdrs = ["min_segment_distance_problem.h"],
    deps = [
        ":qtfm_segment_matcher_v2",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
    ],
)

cc_library(
    name = "quad_tree",
    hdrs = ["quad_tree.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math/geometry:aabox2d",
    ],
)

cc_library(
    name = "quad_tree_field_map",
    hdrs = ["quad_tree_field_map.h"],
    deps = [
        ":quad_tree",
        "//onboard/math/geometry:grid_frame",
    ],
)

cc_library(
    name = "qtfm_segment_matcher_util",
    srcs = ["qtfm_segment_matcher_util.cc"],
    hdrs = ["qtfm_segment_matcher_util.h"],
    deps = [
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:segment2d",
    ],
)

cc_library(
    name = "qtfm_segment_matcher_v2",
    srcs = ["qtfm_segment_matcher_v2.cc"],
    hdrs = ["qtfm_segment_matcher_v2.h"],
    deps = [
        ":qtfm_segment_matcher_util",
        ":quad_tree_field_map",
        "//onboard/container:small_vector",
        "//onboard/math/geometry:grid_frame_util",
        "//onboard/math/geometry:line_intersection",
        "//onboard/math/geometry:segment2d",
        "//onboard/utils:map_util",
        "//onboard/vis/canvas",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "prediction_util_test",
    srcs = ["prediction_util_test.cc"],
    deps = [
        ":prediction_util",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:test_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "perception_util_test",
    srcs = ["perception_util_test.cc"],
    deps = [
        ":perception_util",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/math/geometry:util",
        "//onboard/planner/test_util:util",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_geometry_util_test",
    srcs = ["vehicle_geometry_util_test.cc"],
    deps = [
        ":vehicle_geometry_util",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/math:vec",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_util_test",
    srcs = ["path_util_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/util:path_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "min_segment_distance_problem_test",
    srcs = ["min_segment_distance_problem_test.cc"],
    deps = [
        ":min_segment_distance_problem",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "quad_tree_test",
    srcs = ["quad_tree_test.cc"],
    deps = [
        ":quad_tree",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "qtfm_segment_matcher_v2_test",
    srcs = ["qtfm_segment_matcher_v2_test.cc"],
    deps = [
        ":qtfm_segment_matcher_v2",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "qtfm_segment_matcher_util_test",
    srcs = ["qtfm_segment_matcher_util_test.cc"],
    deps = [
        ":qtfm_segment_matcher_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
