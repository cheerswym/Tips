load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "st_boundary_pre_decider",
    srcs = ["st_boundary_pre_decider.cc"],
    hdrs = ["st_boundary_pre_decider.h"],
    deps = [
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/speed:st_boundary_with_decision",
    ],
)

cc_library(
    name = "close_object_slowdown_decider",
    srcs = ["close_object_slowdown_decider.cc"],
    hdrs = ["close_object_slowdown_decider.h"],
    deps = [
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/speed:st_graph",
        "//onboard/planner/util:path_util",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "post_st_boundary_modifier",
    srcs = ["post_st_boundary_modifier.cc"],
    hdrs = ["post_st_boundary_modifier.h"],
    deps = [
        ":st_boundary_modifier_util",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/object:plot_util",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/speed:st_boundary",
        "//onboard/planner/speed:st_boundary_with_decision",
        "//onboard/planner/speed:st_graph",
        "//onboard/planner/util:path_util",
        "//onboard/prediction:prediction_defs",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "pre_st_boundary_modifier",
    srcs = ["pre_st_boundary_modifier.cc"],
    hdrs = ["pre_st_boundary_modifier.h"],
    deps = [
        ":st_boundary_modifier_util",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/planner:discretized_path",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/speed:st_boundary_with_decision",
        "//onboard/planner/speed:st_graph",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
    ],
)

cc_library(
    name = "st_boundary_modifier_util",
    srcs = ["st_boundary_modifier_util.cc"],
    hdrs = ["st_boundary_modifier_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/speed:st_boundary_with_decision",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/prediction:predicted_trajectory",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
