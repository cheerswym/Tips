load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_classifier",
    srcs = ["traffic_light_classifier.cc"],
    hdrs = ["traffic_light_classifier.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:vehicle_pose",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/nets:tl_net_classifier",
        "//onboard/params:param_manager",
        "//onboard/perception:projection_util",
        "//onboard/perception:semantic_segmentation_result",
        "//onboard/perception/traffic_light/hmm",
        "//onboard/utils:history_buffer",
        "//onboard/vis/canvas",
        "@opencv",
    ],
)

cc_library(
    name = "traffic_light_decider",
    srcs = ["traffic_light_decider.cc"],
    hdrs = ["traffic_light_decider.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_defs",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cpplint()
