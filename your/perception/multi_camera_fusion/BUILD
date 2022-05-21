load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "track",
    srcs = ["track.cc"],
    hdrs = ["track.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/perception/tracker:track",
        "//onboard/perception/tracker/motion_filter_2:estimator",
        "//onboard/perception/tracker/motion_filter_2:img_2d_kalman_filter",
        "//onboard/perception/tracker/motion_filter_2:img_2d_model",
        "//onboard/utils:history_buffer",
        "@boost//:math",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "single_camera_tracker",
    srcs = ["single_camera_tracker.cc"],
    hdrs = ["single_camera_tracker.h"],
    data = ["//onboard/perception/multi_camera_fusion/data:motion_filter_param"],
    deps = [
        ":track",
        ":tracker_constants",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/perception/tracker/association:association_util",
        "//onboard/perception/tracker/association:onboard_associator",
        "//onboard/perception/tracker/motion_filter_2:meas_model",
    ],
)

cc_library(
    name = "single_camera_tracker_friend",
    hdrs = ["single_camera_tracker_friend.h"],
    deps = [
        ":single_camera_tracker",
        ":tracker_constants",
    ],
)

cc_library(
    name = "tracker_util",
    srcs = ["tracker_util.cc"],
    hdrs = ["tracker_util.h"],
    deps = [
        ":track",
    ],
)

cc_library(
    name = "tracker_constants",
    hdrs = ["tracker_constants.h"],
)

cc_library(
    name = "multi_camera_fusion_engine",
    srcs = ["multi_camera_fusion_engine.cc"],
    hdrs = ["multi_camera_fusion_engine.h"],
    data = ["//onboard/perception/multi_camera_fusion/data:motion_filter_param"],
    deps = [
        ":single_camera_tracker",
        ":tracker_util",
        "//onboard/async:parallel_for",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/params:param_manager",
        "//onboard/perception:projection_util",
        "//onboard/perception/tracker/association:onboard_associator",
    ],
)

cc_library(
    name = "multi_camera_fusion_engine_friend",
    hdrs = ["multi_camera_fusion_engine_friend.h"],
    deps = [
        ":multi_camera_fusion_engine",
        ":tracker_constants",
    ],
)
####################################### TEST ###########################

cc_test(
    name = "single_camera_tracker_test",
    srcs = ["single_camera_tracker_test.cc"],
    deps = [
        ":single_camera_tracker_friend",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "tracker_util_test",
    srcs = ["tracker_util_test.cc"],
    data = ["//onboard/perception/multi_camera_fusion/test_data:pbq_params"],
    deps = [
        ":track",
        ":tracker_util",
        "//onboard/global:test_main",
        "//onboard/math/geometry:util",
        "//onboard/perception/test_util:measurement_builder",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "multi_camera_fusion_engine_test",
    srcs = ["multi_camera_fusion_engine_test.cc"],
    data = ["//onboard/perception/multi_camera_fusion/test_data:pbq_params"],
    deps = [
        ":multi_camera_fusion_engine",
        ":multi_camera_fusion_engine_friend",
        "//onboard/global:test_main",
        "//onboard/perception/test_util:measurement_builder",
        "//onboard/perception/test_util:track_builder",
        "//onboard/utils:test_util",
    ],
)

cpplint()
