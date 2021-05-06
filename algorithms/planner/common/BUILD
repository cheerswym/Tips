load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "path_sl_boundary",
    srcs = ["path_sl_boundary.cc"],
    hdrs = ["path_sl_boundary.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner:planner_defs",
    ],
)

cc_library(
    name = "plot_util",
    srcs = ["plot_util.cc"],
    hdrs = ["plot_util.h"],
    deps = [
        ":path_sl_boundary",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_util",
        "//onboard/vis/common:color",
    ],
)

cc_library(
    name = "lane_path_info",
    srcs = ["lane_path_info.cc"],
    hdrs = ["lane_path_info.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "multi_timer_util",
    srcs = ["multi_timer_util.cc"],
    hdrs = ["multi_timer_util.h"],
    deps = [
        "//onboard/global:timer",
        "//onboard/lite:logging",
        "//onboard/proto:timer_report_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "planner_status",
    srcs = ["planner_status.cc"],
    hdrs = ["planner_status.h"],
    deps = [
        "//onboard/planner/common/proto:planner_status_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "plan_start_point_info",
    hdrs = ["plan_start_point_info.h"],
    deps = [
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "path_approx",
    srcs = ["path_approx.cc"],
    hdrs = ["path_approx.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:offset_rect",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "global_pose",
    srcs = ["global_pose.cc"],
    hdrs = ["global_pose.h"],
    deps = [
        "//common/proto:global_pose_cc_proto",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "agent_overlap",
    srcs = ["agent_overlap.cc"],
    hdrs = ["agent_overlap.h"],
    deps = [
        ":path_approx",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:polygon2d_util",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "path_sl_boundary_test",
    srcs = ["path_sl_boundary_test.cc"],
    deps = [
        ":path_sl_boundary",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lane_path_info_test",
    srcs = ["lane_path_info_test.cc"],
    deps = [
        ":lane_path_info",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "path_approx_test",
    srcs = ["path_approx_test.cc"],
    deps = [
        ":path_approx",
        "//onboard/math:test_util",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "agent_overlap_test",
    srcs = ["agent_overlap_test.cc"],
    deps = [
        ":agent_overlap",
        "//onboard/planner/test_util:util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
