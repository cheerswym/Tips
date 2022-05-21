load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tcn_constant",
    hdrs = ["tcn_constant.h"],
)

cc_library(
    name = "track_classifier",
    srcs = ["track_classifier.cc"],
    hdrs = ["track_classifier.h"],
    deps = [
        ":label_convert_utils",
        ":tcn_constant",
        ":tcn_model",
        ":track_classifier_utils",
        ":track_data_storage",
        "//offboard/labeling/proto:label_frame_cc_proto",
        "//offboard/labeling/proto:track_classifier_data_cc_proto",
        "//offboard/labeling/proto:track_classifier_label_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:parallel_for",
        "//onboard/base",
        "//onboard/camera/utils:camera_image",
        "//onboard/global:car_common",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_manager",
        "//onboard/nets:tcn_net_classifier",
        "//onboard/perception/tracker:tracker_util",
        "//onboard/vis/canvas",
        "@opencv",
    ],
)

cc_library(
    name = "camera_embedding_manager",
    srcs = ["camera_embedding_manager.cc"],
    hdrs = ["camera_embedding_manager.h"],
    deps = [
        ":track_classifier_utils",
        "//offboard/labeling/proto:track_classifier_data_cc_proto",
        "//onboard/base",
        "//onboard/camera/utils:camera_image",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/proto:perception_cc_proto",
        "@com_github_google_leveldb//:leveldb",
        "@opencv",
    ],
)

cc_library(
    name = "laser_embedding_manager",
    srcs = ["laser_embedding_manager.cc"],
    hdrs = ["laser_embedding_manager.h"],
    deps = [
        ":laser_data_storage",
        ":tcn_constant",
        ":track_classifier_utils",
        "//offboard/labeling/proto:track_classifier_data_cc_proto",
        "//onboard/base",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/global:trace",
        "//onboard/math/geometry:polygon2d",
        "//onboard/nets:tcn_net_classifier",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/nets/trt:tcn_net_utils",
        "//onboard/params:param_manager",
        "//onboard/perception:cluster",
        "//onboard/perception:cluster_util",
        "//onboard/perception:projection_util",
        "//onboard/perception/utils:timer",
        "//onboard/proto:perception_cc_proto",
        "@boost//:circular_buffer",
        "@com_github_google_leveldb//:leveldb",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_library(
    name = "track_classifier_utils",
    srcs = ["track_classifier_utils.cc"],
    hdrs = ["track_classifier_utils.h"],
    deps = [
        "//onboard/base",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/perception:projection_util",
        "//onboard/perception/tracker:track",
        "//onboard/proto:perception_cc_proto",
        "@com_github_google_leveldb//:leveldb",
        "@opencv",
    ],
)

cc_library(
    name = "visual_feature_extractor",
    srcs = ["visual_feature_extractor.cc"],
    hdrs = ["visual_feature_extractor.h"],
    deps = [
        "//onboard/base",
        "//onboard/global:trace",
        "//onboard/nets:tcn_net_classifier",
        "//onboard/perception:geometry_util",
        "@opencv",
    ],
)

cc_library(
    name = "label_convert_utils",
    srcs = ["label_convert_utils.cc"],
    hdrs = ["label_convert_utils.h"],
    deps = [
        ":tcn_model",
        "//offboard/labeling/proto:label_frame_cc_proto",
        "//offboard/labeling/proto:track_classifier_label_cc_proto",
        "//onboard/lite:logging",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "data_storage",
    srcs = ["data_storage.cc"],
    hdrs = ["data_storage.h"],
    deps = [
        ":track_classifier_utils",
        "//onboard/lite:logging",
        "@com_github_google_leveldb//:leveldb",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "track_data_storage",
    srcs = ["track_data_storage.cc"],
    hdrs = ["track_data_storage.h"],
    deps = [
        ":data_storage",
        ":label_convert_utils",
        ":tcn_model",
        ":track_classifier_utils",
        "//offboard/labeling/proto:label_frame_cc_proto",
        "//offboard/labeling/proto:track_classifier_data_cc_proto",
        "//offboard/labeling/proto:track_classifier_label_cc_proto",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/maps:semantic_map_manager",
        "//onboard/perception:geometry_util",
        "//onboard/perception/tracker:track",
        "//onboard/perception/tracker:tracker_util",
        "//onboard/vis/canvas",
    ],
)

cc_library(
    name = "laser_data_storage",
    srcs = ["laser_data_storage.cc"],
    hdrs = ["laser_data_storage.h"],
    deps = [
        ":data_storage",
        ":track_classifier_utils",
        "//offboard/labeling/proto:track_classifier_data_cc_proto",
        "//onboard/lite:logging",
        "//onboard/perception:laser_point",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_proto_library(
    name = "tcn_model",
    deps = [
        "//onboard/perception/tracker/track_classifier/proto:tcn_model_proto",
    ],
)

cc_test(
    name = "visual_feature_extractor_test",
    srcs = [
        "visual_feature_extractor_test.cc",
    ],
    data = [
        "//onboard/nets:data",
        "//onboard/params/run_params:run_param_data",
    ],
    tags = [
        "exclusive",
        "gpu",
    ],
    deps = [
        ":visual_feature_extractor",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "label_convert_utils_test",
    srcs = ["label_convert_utils_test.cc"],
    tags = [
        "exclusive",
        "gpu",
    ],
    deps = [
        ":label_convert_utils",
        "@com_github_google_benchmark//:benchmark",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
