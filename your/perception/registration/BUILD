load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tic_toc",
    hdrs = [
        "tic_toc.h",
    ],
)

cc_library(
    name = "registration_common_tool",
    srcs = [
        "registration_common_tool.cc",
    ],
    hdrs = [
        "registration_common_tool.h",
    ],
    deps = [
        "//onboard/math:vec",
        "@pcl//:common",
    ],
)

cc_library(
    name = "point_matcher_structs",
    hdrs = [
        "point_matcher_structs.h",
    ],
    deps = [
        "//onboard/math/geometry:affine_transformation",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "icp",
    srcs = ["icp.cc"],
    hdrs = ["icp.h"],
    deps = [
        ":point_matcher_structs",
        ":registration_common_tool",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:kdtree",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "@com_google_absl//absl/strings:str_format",
        "@eigen",
    ],
)

cc_library(
    name = "icp_with_normal",
    srcs = ["icp_with_normal.cc"],
    hdrs = ["icp_with_normal.h"],
    deps = [
        "registration_common_tool",
        ":point_matcher_structs",
        "//offboard/mapping/mapping_core/util:point_cloud_util",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:kdtree",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@eigen",
        "@pcl//:common",
        "@pcl//:kdtree",
        "@pcl//:registration",
    ],
)

cc_library(
    name = "icp_with_normal_lls",
    srcs = ["icp_with_normal_lls.cc"],
    hdrs = ["icp_with_normal_lls.h"],
    deps = [
        ":point_matcher_structs",
        ":registration_common_tool",
        "//offboard/mapping/mapping_core/util:point_cloud_util",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:kdtree",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@eigen",
        "@pcl//:common",
        "@pcl//:kdtree",
        "@pcl//:registration",
    ],
)

cc_library(
    name = "icp_with_covariance",
    srcs = ["icp_with_covariance.cc"],
    hdrs = ["icp_with_covariance.h"],
    deps = [
        ":point_matcher_structs",
        ":registration_common_tool",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:kdtree",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@eigen",
        "@pcl//:common",
    ],
)

cc_library(
    name = "icp_with_normal_util",
    srcs = ["icp_with_normal_util.cc"],
    hdrs = ["icp_with_normal_util.h"],
    deps = [
        "//offboard/mapping/mapping_core/util:point_cloud_util",
        "//onboard/math:vec",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "@boost//:smart_ptr",
        "@pcl//:features",
    ],
)

cc_library(
    name = "icp_tracker",
    srcs = ["icp_tracker.cc"],
    hdrs = ["icp_tracker.h"],
    deps = [
        ":icp",
        ":icp_with_normal_util",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "//onboard/perception:perception_util",
        # "//onboard/perception:tracker_util",
        "//onboard/perception/registration:icp_with_normal_lls",
        "//onboard/perception/registration:icp_with_covariance",
        "//onboard/perception/segmentation:segmenter",
        "//onboard/perception/registration:tic_toc",
        "//onboard/perception/tracker:tracker_util",
    ],
)

cc_test(
    name = "icp_test",
    srcs = ["icp_test.cc"],
    deps = [
        ":icp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "icp_with_covariance_test",
    srcs = ["icp_with_covariance_test.cc"],
    data = [
        "//onboard/perception/registration:data",
    ],
    deps = [
        ":icp_with_covariance",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
        "@pcl//:features",
        "@pcl//:io",
    ],
)

filegroup(
    name = "data",
    data = glob(["data/**"]),
)

cpplint()
