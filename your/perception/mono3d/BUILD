load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mono3d_adapter",
    srcs = ["mono3d_adapter.cc"],
    hdrs = ["mono3d_adapter.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/camera:camera_params",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:coordinate_converter",
        "//onboard/nets:mono3d_detector",
        "//onboard/nets:net_util",
        "//onboard/params:param_manager",
        "//onboard/perception:projection_util",
        "//onboard/utils:history_buffer",
        "//onboard/vis/canvas",
        "@opencv",
    ],
)

cpplint()
