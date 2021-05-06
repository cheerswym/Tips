load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Libraries.

cc_library(
    name = "planner_test_setup",
    testonly = True,
    srcs = ["planner_test_setup.cc"],
    hdrs = ["planner_test_setup.h"],
    deps = [
        "//onboard/lite:lite_module",
        "//onboard/maps:lane_point",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:composite_lane_path",
        "//onboard/planner:planner_input",
        "//onboard/planner:teleop_state",
        "//onboard/planner/proto:planner_output_cc_proto",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "perception_object_builder",
    testonly = True,
    srcs = ["perception_object_builder.cc"],
    hdrs = ["perception_object_builder.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "predicted_trajectory_builder",
    testonly = True,
    srcs = ["predicted_trajectory_builder.cc"],
    hdrs = ["predicted_trajectory_builder.h"],
    deps = [
        "//onboard/planner:planner_defs",
        "//onboard/prediction",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "@com_google_absl//absl/memory",
    ],
)

cc_library(
    name = "object_prediction_builder",
    testonly = True,
    srcs = ["object_prediction_builder.cc"],
    hdrs = ["object_prediction_builder.h"],
    deps = [
        ":predicted_trajectory_builder",
        ":util",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/util:prediction_util",
        "//onboard/prediction",
    ],
)

cc_library(
    name = "planner_object_builder",
    testonly = True,
    srcs = ["planner_object_builder.cc"],
    hdrs = ["planner_object_builder.h"],
    deps = [
        ":object_prediction_builder",
        ":util",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:second_order_trajectory_point",
        "//onboard/planner/object:planner_object",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "route_builder",
    testonly = True,
    srcs = ["route_builder.cc"],
    hdrs = ["route_builder.h"],
    data = [
        "//onboard/planner/router/params:param_files",
    ],
    deps = [
        "//onboard/planner:composite_lane_path",
        "//onboard/planner/router:route",
        "//onboard/planner/router:route_engine",
        "//onboard/planner/router:router_flags",
        "//onboard/planner/router/proto:route_params_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/utils:file_util",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    data = [
        "//onboard/planner/params:param_files",
    ],
    deps = [
        "//onboard/math:vec",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:proto_util",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "perception_object_builder_test",
    srcs = ["perception_object_builder_test.cc"],
    deps = [
        ":perception_object_builder",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "predicted_trajectory_builder_test",
    srcs = ["predicted_trajectory_builder_test.cc"],
    deps = [
        ":predicted_trajectory_builder",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_prediction_builder_test",
    srcs = ["object_prediction_builder_test.cc"],
    deps = [
        ":object_prediction_builder",
        ":perception_object_builder",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [
        ":util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_object_builder_test",
    srcs = ["planner_object_builder_test.cc"],
    deps = [
        ":perception_object_builder",
        ":planner_object_builder",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "route_builder_test",
    srcs = ["route_builder_test.cc"],
    deps = [
        ":route_builder",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
