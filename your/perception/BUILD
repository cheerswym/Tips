load("//bazel:common.bzl", "if_cuda")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "laser_point",
    hdrs = ["laser_point.h"],
    deps = [
        "//onboard/lidar:intensity_util",
        "//onboard/math:vec",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
    ],
)

cc_library(
    name = "semantic_segmentation_result",
    srcs = ["semantic_segmentation_result.cc"],
    hdrs = ["semantic_segmentation_result.h"],
    deps = [
        "//onboard/camera:camera_params",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:map_util",
        "@opencv",
    ],
)

cc_library(
    name = "obstacle_constants",
    hdrs = ["obstacle_constants.h"],
)

cc_library(
    name = "obstacle",
    srcs = ["obstacle.cc"],
    hdrs = ["obstacle.h"],
    deps = [
        ":laser_point",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/proto:perception_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "obstacle_util",
    srcs = ["obstacle_util.cc"],
    hdrs = ["obstacle_util.h"],
    deps = [
        ":obstacle",
        "//onboard/maps:map_selector",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/params:vehicle_param_api",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "semantic_map_util",
    srcs = ["semantic_map_util.cc"],
    hdrs = ["semantic_map_util.h"],
    deps = [
        ":obstacle",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/lidar:vehicle_pose",
        "//onboard/maps:maps_helper",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
    ],
)

cc_library(
    name = "cluster",
    srcs = ["cluster.cc"],
    hdrs = ["cluster.h"],
    deps = [
        ":obstacle",
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "@pcl//:common",
    ],
)

cc_library(
    name = "cluster_util",
    srcs = ["cluster_util.cc"],
    hdrs = ["cluster_util.h"],
    deps = [
        ":cluster",
        ":obstacle_util",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math/geometry:polygon2d",
    ],
)

cc_library(
    name = "type_util",
    srcs = ["type_util.cc"],
    hdrs = ["type_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "projection_util",
    srcs = ["projection_util.cc"],
    hdrs = ["projection_util.h"],
    deps = [
        "//onboard/camera:camera_params",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:check",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:camera_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "pose_correction",
    srcs = ["pose_correction.cc"],
    hdrs = ["pose_correction.h"],
    deps = [
        ":laser_point",
        ":obstacle_grid",
        ":semantic_map_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lidar:vehicle_pose_util",
        "//onboard/lite:logging",
        "//onboard/maps:local_imagery",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:util",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/perception/registration:icp",
        "//onboard/perception/registration:icp_with_normal",
        "//onboard/perception/registration:registration_common_tool",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "retroreflector",
    srcs = ["retroreflector.cc"],
    hdrs = ["retroreflector.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/proto:lidar_cc_proto",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_library(
    name = "retroreflector_detector",
    srcs = ["retroreflector_detector.cc"],
    hdrs = ["retroreflector_detector.h"],
    deps = [
        ":retroreflector",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:local_imagery",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/perception/range_image",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_library(
    name = "obstacle_detector",
    srcs = ["obstacle_detector.cc"],
    hdrs = ["obstacle_detector.h"],
    deps = [
        ":obstacle",
        ":obstacle_constants",
        ":obstacle_grid",
        ":obstacle_manager",
        ":obstacle_util",
        ":pose_correction",
        ":semantic_map_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/maps:imagery_manager",
        "//onboard/maps:local_imagery",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/math:util",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/proto:mapping_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@s2geometry",
    ],
)

cc_library(
    name = "obstacle_grid",
    hdrs = ["obstacle_grid.h"],
    deps = [
        ":obstacle",
        "//onboard/async:async_util",
        "//onboard/async:thread_pool",
        "//onboard/global:spin_lock",
        "//onboard/global:trace",
    ],
)

cc_library(
    name = "obstacle_semantic_manager",
    srcs = ["obstacle_semantic_manager.cc"],
    hdrs = ["obstacle_semantic_manager.h"],
    deps = [
        ":obstacle",
        ":obstacle_manager",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/utils:map_util",
        "//onboard/vis/canvas",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_library(
    name = "obstacle_manager",
    srcs = ["obstacle_manager.cc"],
    hdrs = ["obstacle_manager.h"],
    deps = [
        ":cluster",
        ":cluster_util",
        ":obstacle",
        ":obstacle_grid",
        ":obstacle_util",
        ":projection_util",
        ":semantic_map_util",
        "//offboard/labeling/proto:filtering_cc_proto",
        "//offboard/labeling/proto:label_frame_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/camera/utils:image_util",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lidar:intensity_util",
        "//onboard/lidar:vehicle_pose",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:segment2d",
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/nets:mist_obstacle_net",
        "//onboard/nets:mof_net_util",
        "//onboard/nets:panonet_config",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "obstacle_clusterer",
    srcs = ["obstacle_clusterer.cc"],
    hdrs = ["obstacle_clusterer.h"],
    deps = [
        ":cluster",
        ":cluster_util",
        ":obstacle_grid",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_library(
    name = "depth_map",
    srcs = ["depth_map.cc"],
    hdrs = ["depth_map.h"],
    deps = [
        ":laser_point",
        ":projection_util",
        "//onboard/camera/utils:camera_util",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:check",
        "//onboard/params/utils:param_util",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/proto:image_overlay_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "tracker_module",
    srcs = ["tracker_module.cc"],
    hdrs = ["tracker_module.h"],
    deps = [
        ":projection_util",
        "//offboard/labeling:label_frame_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/lidar:vehicle_pose_util",
        "//onboard/lite:logging",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/perception:type_util",
        "//onboard/perception/tracker",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "cluster_filter",
    srcs = ["cluster_filter.cc"],
    hdrs = ["cluster_filter.h"],
    deps = [
        ":cluster",
        ":cluster_util",
        ":obstacle_manager",
        ":obstacle_util",
        "//offboard/labeling/proto:filtering_cc_proto",
        "//offboard/labeling/proto:label_frame_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lidar:intensity_util",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/math/geometry:kdtree",
        "//onboard/nets:mist_net",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/vis/common:colormap",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@pcl//:features",
    ],
)

cc_library(
    name = "cluster_observer",
    srcs = ["cluster_observer.cc"],
    hdrs = ["cluster_observer.h"],
    deps = [
        ":cluster",
        ":cluster_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:car_common",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/perception/range_image",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "vision_module",
    srcs = ["vision_module.cc"],
    hdrs = ["vision_module.h"],
    data = [
        "//offboard/calibration/data:camera_intrinsics_file",
        "//offboard/calibration/data:lidar_camera_extrinsics_file",
    ],
    deps = [
        ":cluster_util",
        ":obstacle",
        ":projection_util",
        "@com_github_gflags_gflags//:gflags",
        "//onboard/maps:local_imagery",
        "//offboard/vis/ark/ark_server:ark_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_update_client",
        "//onboard/async:async_util",
        "//onboard/camera:camera_params",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose_util",
        "//onboard/lite:lite_module",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/localization:localizer",
        "//onboard/maps:semantic_map_io",
        "//onboard/math:coordinate_converter",
        "//onboard/nets:panonet",
        "//onboard/global:run_context",
        "//onboard/nets:depthnet",
        "//onboard/nets:visual_net_classifier",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/perception/mono3d:mono3d_adapter",
        "//onboard/perception/lane_detector:lane_detector_adapter",
        "//onboard/perception/multi_camera_fusion:multi_camera_fusion_engine",
        "//onboard/perception/tracker/track_classifier",
        "//onboard/perception/tracker/track_classifier:camera_embedding_manager",
        "//onboard/perception/tracker/track_classifier:visual_feature_extractor",
        "//onboard/perception/traffic_light:traffic_light_classifier",
        "//onboard/perception/traffic_light:traffic_light_decider",
        "//onboard/proto:image_overlay_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "//onboard/vis/common:colormap",
        "@boost//:circular_buffer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
    ] + if_cuda([
        "@cuda",
    ]),
)

cc_library(
    name = "perception_module",
    srcs = ["perception_module.cc"],
    hdrs = ["perception_module.h"],
    deps = [
        ":cluster_filter",
        ":cluster_observer",
        ":cluster_util",
        ":human_pipeline_manager",
        ":measurement_util",
        ":obstacle_clusterer",
        ":obstacle_detector",
        ":obstacle_manager",
        ":obstacle_semantic_manager",
        ":perception_util",
        ":projection_util",
        ":retroreflector_detector",
        "//offboard/labeling:label_frame_util",
        "//offboard/labeling/proto:filtering_cc_proto",
        "//offboard/mapping:mapping_util",
        "//offboard/vis/ark/ark_server:ark_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_update_client",
        "//onboard/async:async_util",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/global:car_common",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/lidar:lidar_util",
        "//onboard/lidar:spin_reader_test_util",
        "//onboard/lidar:spin_util",
        "//onboard/lidar:vehicle_pose_util",
        "//onboard/lite:lite_module",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/localization:localizer",
        "//onboard/maps:imagery_manager",
        "//onboard/maps:local_imagery",
        "//onboard/maps:semantic_map_io",
        "//onboard/maps:semantic_map_manager",
        "//onboard/nets:ace_associator",
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/nets:fiery_eye_net_classifier_constants",
        "//onboard/nets:image_patch_classifier",
        "//onboard/nets:visual_net_classifier",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/perception/ground_removal:gradient_based_ground_remover",
        "//onboard/perception/range_image",
        "//onboard/perception/range_image:range_image_util",
        "//onboard/perception/registration:icp_tracker",
        "//onboard/perception/segmentation:fiery_eye_net_proposer",
        "//onboard/perception/segmentation:segmenter",
        "//onboard/perception/sensor_fov:sensor_fov_builder",
        "//onboard/perception/tracker/track_classifier",
        "//onboard/perception/tracker/track_classifier:laser_embedding_manager",
        "//onboard/perception/traffic_light:traffic_light_classifier",
        "//onboard/perception/utils:point_filtering_utils",
        "//onboard/proto:image_overlay_cc_proto",
        "//onboard/proto:radar_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "//onboard/vis/common:colormap",
        "@boost//:circular_buffer",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "perception_util",
    srcs = ["perception_util.cc"],
    hdrs = ["perception_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/params:param_manager",
        "//onboard/params/utils:param_util",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "human_pipeline_tracker",
    srcs = ["human_pipeline_tracker.cc"],
    hdrs = ["human_pipeline_tracker.h"],
    deps = [
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/nets:image_patch_classifier",
        "//onboard/nets:net_util",
        "//onboard/perception/tracker:tracker_util",
        "@com_github_gflags_gflags//:gflags",
    ],
)

filegroup(
    name = "human_pipeline_tracker_testdata",
    data = glob([
        "testdata/human_pipeline_tracker/*",
        "testdata/human_pipeline_tracker/*.jpg",
    ]),
)

cc_test(
    name = "human_pipeline_tracker_test",
    srcs = [
        "human_pipeline_tracker_test.cc",
    ],
    data = [
        ":human_pipeline_tracker_testdata",
    ],
    deps = [
        ":human_pipeline_tracker",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "human_pipeline_manager",
    srcs = ["human_pipeline_manager.cc"],
    hdrs = ["human_pipeline_manager.h"],
    deps = [
        ":human_pipeline_tracker",
        ":projection_util",
        "//offboard/labeling/proto:patch_classifier_data_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_canvas_client",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_update_client",
        "//onboard/async:async_util",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/nets:image_patch_classifier",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/perception/utils:sensor_view_utils",
        "//onboard/utils:map_util",
        "//onboard/vis/common:colormap",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "measurement_util",
    srcs = ["measurement_util.cc"],
    hdrs = ["measurement_util.h"],
    deps = [
        "geometry_util",
        ":projection_util",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/params/run_params/proto:run_params_cc_proto",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "geometry_util",
    srcs = ["geometry_util.cc"],
    hdrs = ["geometry_util.h"],
    deps = [
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:util",
        "//onboard/math/geometry:polygon2d",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "pillar_rc_coord_converter",
    srcs = ["pillar_rc_coord_converter.cc"],
    hdrs = ["pillar_rc_coord_converter.h"],
    deps = [
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "pillar_obstacle_converter",
    srcs = ["pillar_obstacle_converter.cc"],
    hdrs = ["pillar_obstacle_converter.h"],
    deps = [
        ":obstacle",
        ":obstacle_grid",
        ":pillar_rc_coord_converter",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/nets:fiery_eye_net_classifier_constants",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@opencv",
    ],
)

cc_binary(
    name = "obstacle_clusterer_bm",
    srcs = [
        "obstacle_clusterer_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        ":obstacle_clusterer",
        ":obstacle_manager",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/random",
    ],
)

cc_binary(
    name = "obstacle_manager_bm",
    srcs = ["obstacle_manager_bm.cc"],
    tags = ["bm"],
    deps = [
        ":obstacle_manager",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_test(
    name = "obstacle_clusterer_test",
    srcs = [
        "obstacle_clusterer_test.cc",
    ],
    deps = [
        ":obstacle_clusterer",
        ":obstacle_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "obstacle_detector_test",
    srcs = [
        "obstacle_detector_test.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cluster_filter",
        ":obstacle_detector",
        "//onboard/lidar:spin_reader_test_util",
        "//onboard/params:param_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "projection_util_test",
    srcs = [
        "projection_util_test.cc",
    ],
    deps = [
        ":projection_util",
        "//onboard/params/utils:param_util",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "semantic_map_util_test",
    srcs = [
        "semantic_map_util_test.cc",
    ],
    deps = [
        ":semantic_map_util",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "obstacle_test",
    srcs = ["obstacle_test.cc"],
    deps = [
        ":obstacle",
        "//onboard/perception/test_util:obstacle_builder",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "obstacle_util_test",
    srcs = ["obstacle_util_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":obstacle_util",
        "//onboard/params:param_manager",
        "//onboard/perception/test_util:obstacle_builder",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "obstacle_util_bm",
    srcs = ["obstacle_util_bm.cc"],
    tags = ["bm"],
    deps = [
        ":obstacle_util",
        "//onboard/math:piecewise_linear_function",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_test(
    name = "cluster_util_test",
    srcs = ["cluster_util_test.cc"],
    deps = [
        ":cluster_util",
        "//onboard/math:test_util",
        "//onboard/perception/test_util:cluster_builder",
        "//onboard/perception/test_util:obstacle_builder",
        "//onboard/utils:test_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "perception_module_test",
    srcs = ["perception_module_test.cc"],
    deps = [
        ":perception_module",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "type_util_test",
    srcs = ["type_util_test.cc"],
    deps = [
        ":type_util",
        "//onboard/math:test_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "perception_util_test",
    srcs = ["perception_util_test.cc"],
    deps = [
        ":perception_util",
        "//onboard/proto:lidar_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
