load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "types",
    srcs = ["types.cc"],
    hdrs = ["types.h"],
    deps = [
        "//onboard/lidar:spin_structs",
        "//onboard/maps:local_imagery",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:polygon2d",
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/perception:cluster",
        "//onboard/perception:human_pipeline_manager",
        "//onboard/perception:obstacle_manager",
        "//onboard/perception:obstacle_util",
        "//onboard/perception:retroreflector",
        "//onboard/perception:semantic_map_util",
        "//onboard/perception/range_image",
        "//onboard/proto:camera_cc_proto",
        "//onboard/proto:lidar_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "proposed_cluster",
    srcs = ["proposed_cluster.cc"],
    hdrs = ["proposed_cluster.h"],
    deps = [
        ":types",
        "//onboard/perception:cluster",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "proposed_cluster_builder",
    testonly = True,
    srcs = ["proposed_cluster_builder.cc"],
    hdrs = ["proposed_cluster_builder.h"],
    deps = [
        ":proposed_cluster",
        "//onboard/lite:logging",
        "//onboard/perception/test_util:cluster_builder",
    ],
)

cc_library(
    name = "proposer_util",
    srcs = ["proposer_util.cc"],
    hdrs = ["proposer_util.h"],
    deps = [
        ":proposed_cluster",
        "//onboard/math:util",
        "//onboard/perception:obstacle_util",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "proposer_tree",
    srcs = ["proposer_tree.cc"],
    hdrs = ["proposer_tree.h"],
    data = glob([
        "conf/*.pb.txt",
    ]),
    deps = [
        ":proposed_cluster",
        ":proposer",
        ":scene_map",
        ":segmenter_config_cc_proto",
        "//onboard/async:parallel_for",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "proposer_tree_util",
    srcs = ["proposer_tree_util.cc"],
    hdrs = ["proposer_tree_util.h"],
    deps = [
        ":barrier_proposer",
        ":blooming_proposer",
        ":bpearl_noise_proposer",
        ":cateye_proposer",
        ":distant_cluster_proposer",
        ":fiery_eye_net_proposer",
        ":large_vehicle_proposer",
        ":leaf_proposer",
        ":ll_net_proposer",
        ":ouster_noise_proposer",
        ":proposer_tree",
        ":reflection_proposer",
        ":segmenter_config_cc_proto",
        ":semantic_map_proposer",
        ":splash_proposer",
    ],
)

cc_library(
    name = "scene_map",
    hdrs = ["scene_map.h"],
    deps = [
        ":proposed_cluster",
        "//onboard/async:async_util",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "scene_map_test_util",
    testonly = True,
    srcs = ["scene_map_test_util.cc"],
    hdrs = ["scene_map_test_util.h"],
    deps = [
        ":scene_map",
        "//onboard/lite:logging",
        "//onboard/perception/test_util:cluster_builder",
    ],
)

cc_library(
    name = "proposed_cluster_dag",
    srcs = ["proposed_cluster_dag.cc"],
    hdrs = ["proposed_cluster_dag.h"],
    deps = [
        ":proposed_cluster",
        ":scene_map",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "proposer",
    hdrs = ["proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer_util",
        ":types",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "proposer_test_util",
    testonly = True,
    srcs = ["proposer_test_util.cc"],
    hdrs = ["proposer_test_util.h"],
    deps = [
        ":proposed_cluster",
        ":proposed_cluster_builder",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/perception:cluster_util",
    ],
)

cc_library(
    name = "fiery_eye_net_proposer",
    srcs = ["fiery_eye_net_proposer.cc"],
    hdrs = ["fiery_eye_net_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:trace",
        "//onboard/perception:cluster_util",
        "//onboard/utils:map_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "leaf_proposer",
    srcs = ["leaf_proposer.cc"],
    hdrs = ["leaf_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/perception:cluster_util",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "ll_net_proposer",
    srcs = ["ll_net_proposer.cc"],
    hdrs = ["ll_net_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/camera/utils:image_util",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:point_types",
        "//onboard/math:util",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:kdtree",
        "//onboard/nets:panonet_config",
        "//onboard/params/utils:param_util",
        "//onboard/perception:cluster_util",
        "//onboard/perception:obstacle_util",
        "//onboard/perception:projection_util",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings",
        "@opencv",
        "@pcl//:segmentation",
    ],
)

cc_library(
    name = "ouster_noise_proposer",
    srcs = ["ouster_noise_proposer.cc"],
    hdrs = ["ouster_noise_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math/geometry:segment2d",
        "//onboard/perception:cluster_util",
    ],
)

cc_library(
    name = "cateye_proposer",
    srcs = ["cateye_proposer.cc"],
    hdrs = ["cateye_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/perception:cluster_util",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "distant_cluster_proposer",
    srcs = ["distant_cluster_proposer.cc"],
    hdrs = ["distant_cluster_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/perception:cluster_util",
        "//onboard/perception:semantic_map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "bpearl_noise_proposer",
    srcs = ["bpearl_noise_proposer.cc"],
    hdrs = ["bpearl_noise_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
    ],
)

cc_library(
    name = "blooming_proposer",
    srcs = ["blooming_proposer.cc"],
    hdrs = ["blooming_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/mapping/mapping_core/util:point_cloud_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math:util",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:kdtree",
        "//onboard/math/geometry:polygon2d",
        "//onboard/perception:cluster_util",
        "//onboard/perception:obstacle_util",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
        "@pcl//:segmentation",
    ],
)

cc_library(
    name = "semantic_map_proposer",
    srcs = ["semantic_map_proposer.cc"],
    hdrs = ["semantic_map_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math:util",
        "//onboard/perception:cluster_util",
        "//onboard/perception:semantic_map_util",
    ],
)

cc_library(
    name = "barrier_proposer",
    srcs = ["barrier_proposer.cc"],
    hdrs = ["barrier_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        "//offboard/mapping/mapping_core/util:point_cloud_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math:util",
        "@pcl//:segmentation",
    ],
)

cc_library(
    name = "large_vehicle_proposer",
    srcs = ["large_vehicle_proposer.cc"],
    hdrs = ["large_vehicle_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        ":proposer_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math/geometry:kdtree",
        "//onboard/perception:obstacle_util",
        "//onboard/perception:projection_util",
        "//onboard/perception:semantic_segmentation_result",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "reflection_proposer",
    srcs = ["reflection_proposer.cc"],
    hdrs = ["reflection_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
        ":proposer_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/perception:obstacle_util",
        "//onboard/perception:semantic_map_util",
        "//onboard/perception/range_image",
    ],
)

cc_library(
    name = "splash_proposer",
    srcs = ["splash_proposer.cc"],
    hdrs = ["splash_proposer.h"],
    deps = [
        ":proposed_cluster",
        ":proposer",
    ],
)

cc_library(
    name = "pyramid",
    srcs = ["pyramid.cc"],
    hdrs = ["pyramid.h"],
    deps = [
        ":proposed_cluster_dag",
        ":proposer_tree",
        ":scene_map",
    ],
)

cc_library(
    name = "segmenter",
    srcs = ["segmenter.cc"],
    hdrs = ["segmenter.h"],
    deps = [
        ":proposed_cluster",
        ":proposed_cluster_dag",
        ":proposer_tree",
        ":proposer_tree_util",
        ":proposer_util",
        ":pyramid",
        ":scene_map",
        ":segmenter_config_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/lite:logging",
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "//onboard/perception:obstacle_manager",
        "//onboard/perception/range_image",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
)

###############################################################################
# Protos.

proto_library(
    name = "segmenter_config_proto",
    srcs = ["proto/segmenter_config.proto"],
)

cc_proto_library(
    name = "segmenter_config_cc_proto",
    deps = [":segmenter_config_proto"],
)

###############################################################################
# Tests.

cc_test(
    name = "proposer_tree_util_test",
    srcs = ["proposer_tree_util_test.cc"],
    deps = [
        ":proposer_tree_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "proposer_util_test",
    srcs = ["proposer_util_test.cc"],
    deps = [
        ":proposer_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "proposed_cluster_dag_test",
    srcs = ["proposed_cluster_dag_test.cc"],
    deps = [
        ":proposed_cluster_dag",
        ":scene_map_test_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "fiery_eye_net_proposer_test",
    srcs = ["fiery_eye_net_proposer_test.cc"],
    deps = [
        ":fiery_eye_net_proposer",
        ":proposer_test_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "pyramid_test",
    srcs = ["pyramid_test.cc"],
    deps = [
        ":proposer_tree_util",
        ":pyramid",
        ":scene_map_test_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
