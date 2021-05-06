load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "planner_object",
    srcs = ["planner_object.cc"],
    hdrs = ["planner_object.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/util:perception_util",
        "//onboard/prediction",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "object_vector",
    hdrs = ["object_vector.h"],
    deps = [
        "//onboard/container:strong_vector",
    ],
)

cc_library(
    name = "spacetime_object_state",
    srcs = ["spacetime_object_state.cc"],
    hdrs = ["spacetime_object_state.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:planner_defs",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
    ],
)

cc_library(
    name = "spacetime_object_trajectory",
    srcs = ["spacetime_object_trajectory.cc"],
    hdrs = ["spacetime_object_trajectory.h"],
    deps = [
        ":spacetime_object_state",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/object:planner_object",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "low_likelihood_filter",
    srcs = ["low_likelihood_filter.cc"],
    hdrs = ["low_likelihood_filter.h"],
    deps = [
        ":trajectory_filter",
        "//onboard/planner/object:planner_object",
        "//onboard/prediction:predicted_trajectory",
    ],
)

cc_library(
    name = "spacetime_trajectory_manager",
    srcs = ["spacetime_trajectory_manager.cc"],
    hdrs = ["spacetime_trajectory_manager.h"],
    deps = [
        ":spacetime_object_trajectory",
        ":spacetime_planner_trajectory_finder",
        ":trajectory_filter",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/planner:planner_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/utils:map_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_object_manager",
    srcs = ["planner_object_manager.cc"],
    hdrs = ["planner_object_manager.h"],
    deps = [
        "//onboard/planner/object:object_vector",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/util:prediction_util",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_object_manager_builder",
    srcs = ["planner_object_manager_builder.cc"],
    hdrs = ["planner_object_manager_builder.h"],
    deps = [
        ":object_vector",
        ":planner_object_manager",
        "//onboard/async:async_macro",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/container:strong_vector",
        "//onboard/global:trace",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/planner/util:prediction_util",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "trajectory_filter",
    hdrs = ["trajectory_filter.h"],
    deps = [
        ":planner_object",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "drive_passage_filter",
    srcs = ["drive_passage_filter.cc"],
    hdrs = ["drive_passage_filter.h"],
    deps = [
        ":trajectory_filter",
        "//onboard/eval:qevent",
        "//onboard/math:util",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/prediction:prediction_util",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "reflected_object_in_proximity_filter",
    srcs = ["reflected_object_in_proximity_filter.cc"],
    hdrs = ["reflected_object_in_proximity_filter.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "motion_state_filter",
    srcs = ["motion_state_filter.cc"],
    hdrs = ["motion_state_filter.h"],
    deps = [
        ":trajectory_filter",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "freespace_region_filter",
    srcs = ["freespace_region_filter.cc"],
    hdrs = ["freespace_region_filter.h"],
    deps = [
        ":trajectory_filter",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/prediction:prediction_util",
    ],
)

cc_library(
    name = "predicted_motion_filter",
    srcs = ["predicted_motion_filter.cc"],
    hdrs = ["predicted_motion_filter.h"],
    deps = [
        "//onboard/math/geometry:aabox_kdtree2d",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:trajectory_filter",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "spacetime_planner_trajectory_finder",
    srcs = ["spacetime_planner_trajectory_finder.cc"],
    hdrs = ["spacetime_planner_trajectory_finder.h"],
    deps = [
        ":spacetime_object_trajectory",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "spacetime_trajectory_manager_builder",
    srcs = ["spacetime_trajectory_manager_builder.cc"],
    hdrs = ["spacetime_trajectory_manager_builder.h"],
    deps = [
        ":drive_passage_filter",
        ":low_likelihood_filter",
        ":planner_object_manager",
        ":spacetime_planner_trajectory_finder",
        ":spacetime_trajectory_manager",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":spacetime_object_trajectory",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/vis/common:color",
    ],
)

cc_library(
    name = "partial_spacetime_object_trajectory",
    hdrs = ["partial_spacetime_object_trajectory.h"],
    deps = [
        ":spacetime_object_trajectory",
        "//onboard/math:range1d",
        "//onboard/math:util",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "spacetime_object_trajectory_test",
    srcs = ["spacetime_object_trajectory_test.cc"],
    deps = [
        ":spacetime_object_trajectory",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/math:test_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:predicted_trajectory_builder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "spacetime_trajectory_manager_test",
    srcs = ["spacetime_trajectory_manager_test.cc"],
    deps = [
        ":spacetime_trajectory_manager",
        ":trajectory_filter",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/math:test_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/util:prediction_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_manager_test",
    srcs = ["planner_object_manager_test.cc"],
    deps = [
        ":planner_object_manager",
        ":planner_object_manager_builder",
        "//onboard/container:strong_vector",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/proto:planner_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_manager_builder_test",
    srcs = ["planner_object_manager_builder_test.cc"],
    deps = [
        ":planner_object_manager_builder",
        "//onboard/container:strong_vector",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/proto:planner_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "motion_state_filter_test",
    srcs = ["motion_state_filter_test.cc"],
    deps = [
        ":motion_state_filter",
        "//onboard/global:test_main",
        "//onboard/math:vec",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "predicted_motion_filter_test",
    srcs = ["predicted_motion_filter_test.cc"],
    deps = [
        ":predicted_motion_filter",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/planner/object:object_vector",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/proto:perception_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
