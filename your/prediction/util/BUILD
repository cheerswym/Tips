load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "trajectory_developer",
    srcs = ["trajectory_developer.cc"],
    hdrs = ["trajectory_developer.h"],
    deps = [
        "//onboard/math:line_fitter",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:polynomial_fitter",
        "//onboard/math:vec",
        "//onboard/planner:discretized_path",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/container:prediction_object",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "kinematic_model",
    srcs = ["kinematic_model.cc"],
    hdrs = ["kinematic_model.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math/geometry:util",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "lane_path_finder",
    srcs = ["lane_path_finder.cc"],
    hdrs = ["lane_path_finder.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:vec",
        "//onboard/planner:discretized_path",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "transform_util",
    srcs = ["transform_util.cc"],
    hdrs = ["transform_util.h"],
    deps = [
        "//onboard/math:coordinate_converter",
        "//onboard/math/geometry:util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "trajectory_developer_test",
    srcs = ["trajectory_developer_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/prediction/util:trajectory_developer",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_path_finder_test",
    srcs = ["lane_path_finder_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner:discretized_path",
        "//onboard/prediction/util:lane_path_finder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "transform_util_test",
    srcs = ["transform_util_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/prediction/util:transform_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
