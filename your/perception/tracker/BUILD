load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracker_util",
    srcs = ["tracker_util.cc"],
    hdrs = ["tracker_util.h"],
    deps = [
        ":track",
        ":tracker_constant",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/math:eigen",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/nets:fiery_eye_net_classifier_config",
        "//onboard/perception/multi_camera_fusion:track",
        "//onboard/perception/sensor_fov",
        "//onboard/vis/canvas",
    ],
)

cc_library(
    name = "tracker_constant",
    hdrs = ["tracker_constant.h"],
)

cc_library(
    name = "track",
    srcs = ["track.cc"],
    hdrs = ["track.h"],
    deps = [
        ":tracker_constant",
        ":voter",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/nets:fiery_eye_net_classifier_config",
        "//onboard/perception:cluster_util",
        "//onboard/perception:obstacle",
        "//onboard/perception/tracker/motion_filter_2:estimator",
        "//onboard/utils:history_buffer",
        "@boost//:math",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "track_merge_split_manager",
    srcs = ["track_merge_split_manager.cc"],
    hdrs = ["track_merge_split_manager.h"],
    deps = [
        ":track",
        ":tracker_util",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math/geometry:polygon2d",
    ],
)

cc_library(
    name = "track_life_manager",
    srcs = ["track_life_manager.cc"],
    hdrs = ["track_life_manager.h"],
    deps = [
        ":track",
        ":tracker_util",
        "//onboard/lidar:vehicle_pose",
    ],
)

cc_library(
    name = "motion_static_classifier",
    srcs = ["motion_static_classifier.cc"],
    hdrs = ["motion_static_classifier.h"],
    deps = [
        ":track",
        ":tracker_util",
        "//onboard/lidar:vehicle_pose",
    ],
)

cc_test(
    name = "motion_static_classifier_test",
    srcs = [
        "motion_static_classifier_test.cc",
    ],
    deps = [
        ":motion_static_classifier",
        "//onboard/global:test_main",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:test_util",
    ],
)

cc_library(
    name = "shooter_killer",
    srcs = ["shooter_killer.cc"],
    hdrs = ["shooter_killer.h"],
    deps = [
        ":track",
        ":tracker_util",
        "//onboard/lidar:vehicle_pose",
    ],
)

cc_library(
    name = "shooter_killer_friend",
    hdrs = ["shooter_killer_friend.h"],
    deps = [
        ":shooter_killer",
    ],
)

cc_test(
    name = "shooter_killer_test",
    srcs = [
        "shooter_killer_test.cc",
    ],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":shooter_killer_friend",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:file_util",
        "//onboard/utils:test_util",
    ],
)

cc_library(
    name = "tracker_output_guarder",
    srcs = ["tracker_output_guarder.cc"],
    hdrs = ["tracker_output_guarder.h"],
    deps = [
        ":motion_static_classifier",
        ":shooter_killer",
        ":track",
        ":tracker_util",
        "//onboard/lidar:vehicle_pose",
    ],
)

cc_library(
    name = "tracker",
    srcs = ["tracker.cc"],
    hdrs = ["tracker.h"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":track",
        ":track_life_manager",
        ":track_measurement_source_type_manager",
        ":track_merge_split_manager",
        ":tracker_constant",
        ":tracker_output_guarder",
        ":tracker_util",
        "//offboard/labeling:label_frame_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/nets:fiery_eye_net_classifier_config",
        "//onboard/perception:cluster_util",
        "//onboard/perception:geometry_util",
        "//onboard/perception:obstacle",
        "//onboard/perception:obstacle_constants",
        "//onboard/perception:perception_util",
        "//onboard/perception:type_util",
        "//onboard/perception/segmentation:segmenter",
        "//onboard/perception/tracker/association:onboard_associator",
        "//onboard/perception/tracker/motion_filter_2:meas_model",
        "//onboard/perception/tracker/motion_filter_2:utils",
        "//onboard/perception/tracker/track_classifier",
        "//onboard/perception/utils:vis_utils",
        "//onboard/utils:history_buffer",
        "@boost//:circular_buffer",
        "@boost//:math",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "voter",
    srcs = ["voter.cc"],
    hdrs = ["voter.h"],
    deps = [
        ":tracker_constant",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
    ],
)

cc_test(
    name = "voter_test",
    srcs = [
        "voter_test.cc",
    ],
    deps = [
        ":voter",
        "//onboard/lite:logging",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "track_measurement_source_type_manager",
    srcs = ["track_measurement_source_type_manager.cc"],
    hdrs = ["track_measurement_source_type_manager.h"],
    deps = [
        ":track",
        ":tracker_constant",
        "//onboard/global:car_common",
        "//onboard/global:run_context",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:logging",
        "//onboard/params:param_manager",
        "//onboard/perception:obstacle_constants",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
    ],
)

####################################### TEST ###########################
cc_test(
    name = "track_life_manager_test",
    srcs = [
        "track_life_manager_test.cc",
    ],
    deps = [
        ":track_life_manager",
        "//onboard/global:test_main",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/math/geometry:polygon2d",
        "//onboard/perception/test_util:measurement_builder",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "track_merge_split_manager_test",
    srcs = [
        "track_merge_split_manager_test.cc",
    ],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":track_merge_split_manager",
        ":tracker_util",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/math/geometry:polygon2d",
        "//onboard/perception:geometry_util",
        "//onboard/perception/test_util:measurement_builder",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:file_util",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "track_measurement_source_type_manager_test",
    srcs = [
        "track_measurement_source_type_manager_test.cc",
    ],
    data = ["//onboard/perception/tracker/test_data:pbq_params"],
    deps = [
        ":track_measurement_source_type_manager",
        "//onboard/global:test_main",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/math/geometry:polygon2d",
        "//onboard/perception/test_util:measurement_builder",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:test_util",
    ],
)
