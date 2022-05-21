load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gradient_based_ground_remover",
    srcs = ["gradient_based_ground_remover.cc"],
    hdrs = ["gradient_based_ground_remover.h"],
    deps = [
        ":ground_remover",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_update_client",
        "//onboard/lite:logging",
        "//onboard/math:fast_rsqrt",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@opencv",
    ],
)

cc_library(
    name = "ground_remover",
    srcs = ["ground_remover.cc"],
    hdrs = ["ground_remover.h"],
    deps = [
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lidar:spin_structs",
        "//onboard/params:param_manager",
        "//onboard/perception/range_image",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cpplint()
