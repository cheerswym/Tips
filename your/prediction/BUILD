load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "predicted_trajectory",
    srcs = ["predicted_trajectory.cc"],
    hdrs = ["predicted_trajectory.h"],
    deps = [
        ":prediction_defs",
        "//onboard/lite:logging",
        "//onboard/math/geometry:util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "prediction_defs",
    hdrs = ["prediction_defs.h"],
    deps = ["//onboard/proto:perception_cc_proto"],
)

cc_library(
    name = "prediction",
    srcs = ["prediction.cc"],
    hdrs = ["prediction.h"],
    deps = [
        ":predicted_trajectory",
        ":prediction_defs",
        "//onboard/lite:logging",
        "//onboard/math/geometry:util",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "prediction_flags",
    srcs = ["prediction_flags.cc"],
    hdrs = ["prediction_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "prediction_util",
    srcs = ["prediction_util.cc"],
    hdrs = ["prediction_util.h"],
    deps = [
        ":prediction",
        ":prediction_flags",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/planner/util:trajectory_plot_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
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
        "//onboard/maps:semantic_map_util",
        "//onboard/math:test_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/test_util:object_prediction_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
