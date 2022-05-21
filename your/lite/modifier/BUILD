load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "message_modifier_base",
    srcs = ["message_modifier_base.cc"],
    hdrs = ["message_modifier_base.h"],
    deps = [
        "//offboard/simulation/proto:scenario_cc_proto",
        "//onboard/global:registry",
        "//onboard/lite:lite_message_util",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "message_modifier_man",
    srcs = ["message_modifier_man.cc"],
    hdrs = ["message_modifier_man.h"],
    deps = [
        ":change_traffic_lights",
        ":keep_objects",
        ":message_modifier_base",
        ":remove_objects",
        ":replace_objects",
        ":teleport_sdc",
        "//offboard/simulation/proto:scenario_cc_proto",
        "//onboard/global:registry",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
    alwayslink = 1,
)

cc_library(
    name = "remove_objects",
    hdrs = ["remove_objects.h"],
    deps = [":message_modifier_base"],
    alwayslink = 1,
)

cc_library(
    name = "keep_objects",
    hdrs = ["keep_objects.h"],
    deps = [":message_modifier_base"],
    alwayslink = 1,
)

cc_library(
    name = "change_traffic_lights",
    hdrs = ["change_traffic_lights.h"],
    deps = [
        ":message_modifier_base",
    ],
    alwayslink = 1,
)

cc_library(
    name = "replace_objects",
    srcs = ["replace_objects.cc"],
    hdrs = ["replace_objects.h"],
    deps = [
        ":message_modifier_base",
        "//offboard/agents/proto:sim_agent_cc_proto",
        "//offboard/simulation/proto:scenario_cc_proto",
        "//offboard/simulation/proto:virtual_object_cc_proto",
        "//onboard/global:registry",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/math/geometry/proto:box2d_cc_proto",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:sim_augmentation_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
    alwayslink = 1,
)

cc_library(
    name = "teleport_sdc",
    hdrs = ["teleport_sdc.h"],
    deps = [
        ":message_modifier_base",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
    alwayslink = 1,
)

cc_library(
    name = "create_sim_messages_module",
    srcs = ["create_sim_messages_module.cc"],
    hdrs = ["create_sim_messages_module.h"],
    deps = [
        "//offboard/simulation/proto:scenario_cc_proto",
        "//onboard/lite:lite_module",
        "//onboard/lite:module_config_helper",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "synthesized_traffic_flow_module",
    srcs = ["synthesized_traffic_flow_module.cc"],
    hdrs = ["synthesized_traffic_flow_module.h"],
    deps = [
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:lite_module",
        "//onboard/lite:module_config_helper",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/proto:lite_msg_cc_proto",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "message_modifier_test",
    srcs = ["message_modifier_test.cc"],
    deps = [
        ":message_modifier_man",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
