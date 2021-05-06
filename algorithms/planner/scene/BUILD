load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "scene_understanding_output",
    hdrs = ["scene_understanding_output.h"],
    deps = ["//onboard/planner/scene/proto:scene_understanding_cc_proto"],
)

cc_library(
    name = "scene_understanding",
    srcs = ["scene_understanding.cc"],
    hdrs = ["scene_understanding.h"],
    deps = [
        ":occluded_objects_reasoning",
        ":traffic_flow_reasoning",
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "traffic_flow_reasoning",
    srcs = ["traffic_flow_reasoning.cc"],
    hdrs = ["traffic_flow_reasoning.h"],
    deps = [
        ":scene_understanding_output",
        "//onboard/async:parallel_for",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/util:perception_util",
        "//onboard/prediction:prediction_util",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "objects_reasoning",
    srcs = ["objects_reasoning.cc"],
    hdrs = ["objects_reasoning.h"],
    deps = [
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "off_road_scene_reasoning",
    srcs = ["off_road_scene_reasoning.cc"],
    hdrs = ["off_road_scene_reasoning.h"],
    deps = [
        ":objects_reasoning",
        "//onboard/global:trace",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "scene_reasoning_util",
    srcs = ["scene_reasoning_util.cc"],
    hdrs = ["scene_reasoning_util.h"],
    deps = [
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "occluded_objects_reasoning",
    srcs = ["occluded_objects_reasoning.cc"],
    hdrs = ["occluded_objects_reasoning.h"],
    deps =
        [
            "//onboard/perception/sensor_fov",
            "//onboard/planner:planner_semantic_map_manager",
            "//onboard/planner/router:route_sections",
            "//onboard/planner/router:route_sections_util",
            "//onboard/planner/scene/proto:scene_understanding_cc_proto",
            "@com_google_absl//absl/container:flat_hash_set",
            "@com_google_absl//absl/status:statusor",
        ],
)

###############################################################################
# Tests.
cc_test(
    name = "traffic_flow_reasoning_test",
    srcs = ["traffic_flow_reasoning_test.cc"],
    deps = [
        ":traffic_flow_reasoning",
        "//onboard/planner:planner_flags",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "off_road_scene_reasoning_test",
    srcs = ["off_road_scene_reasoning_test.cc"],
    deps = [
        ":off_road_scene_reasoning",
        "//onboard/planner/object:planner_object_manager_builder",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "occluded_objects_reasoning_test",
    srcs = ["occluded_objects_reasoning_test.cc"],
    deps = [
        ":occluded_objects_reasoning",
        "//onboard/perception/sensor_fov:sensor_fov_test_util",
        "//onboard/perception/test_util:obstacle_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
