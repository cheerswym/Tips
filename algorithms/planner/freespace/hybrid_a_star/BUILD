load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "node_3d",
    srcs = ["node_3d.cc"],
    hdrs = ["node_3d.h"],
    deps = [
        "//onboard/math:fast_math",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "reeds_shepp",
    srcs = ["reeds_shepp.cc"],
    hdrs = ["reeds_shepp.h"],
    deps = [
        ":node_3d",
        "//onboard/math:fast_math",
        "//onboard/math:util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "fast_reeds_shepp",
    srcs = ["fast_reeds_shepp.cc"],
    hdrs = [
        "fast_reeds_shepp.h",
        "fast_reeds_shepp_table.h",
    ],
    deps = [
        ":node_3d",
        ":reeds_shepp",
    ],
)

cc_library(
    name = "a_star_heuristics",
    srcs = ["a_star_heuristics.cc"],
    hdrs = ["a_star_heuristics.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace:path_manager_util",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "hybrid_a_star",
    srcs = ["hybrid_a_star.cc"],
    hdrs = ["hybrid_a_star.h"],
    deps = [
        ":a_star_heuristics",
        ":fast_reeds_shepp",
        ":node_3d",
        ":reeds_shepp",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:trace",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:discretized_path",
        "//onboard/planner/freespace:freespace_planner_defs",
        "//onboard/planner/freespace:path_manager_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
        "@boost//:heap",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
    ],
)

###############################################################################
# Proto.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "reeds_shepp_test",
    srcs = ["reeds_shepp_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":reeds_shepp",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "hybrid_a_star_test",
    srcs = ["hybrid_a_star_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":hybrid_a_star",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
