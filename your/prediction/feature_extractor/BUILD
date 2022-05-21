load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "prophnet_feature_extractor",
    srcs = ["prophnet_feature_extractor.cc"],
    hdrs = ["prophnet_feature_extractor.h"],
    deps = [
        ":feature_extraction_util",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:util",
        "//onboard/nets/trt:prophnet",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/proto:perception_cc_proto",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "vehicle_tnt_feature_extractor",
    srcs = ["vehicle_tnt_feature_extractor.cc"],
    hdrs = ["vehicle_tnt_feature_extractor.h"],
    deps = [
        ":feature_extraction_util",
        ":vehicle_tnt_feature",
        "//onboard/maps:semantic_map_manager",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "object_history_sampler",
    srcs = ["object_history_sampler.cc"],
    hdrs = ["object_history_sampler.h"],
    deps = [
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:polynomial_fitter",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history_span",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "feature_extraction_util",
    srcs = ["feature_extraction_util.cc"],
    hdrs = ["feature_extraction_util.h"],
    deps = [
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:polynomial_fitter",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "vehicle_tnt_feature",
    hdrs = ["vehicle_tnt_feature.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/proto:prediction_cc_proto",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "feature_extraction_util_test",
    srcs = ["feature_extraction_util_test.cc"],
    deps = [
        ":feature_extraction_util",
        "//onboard/global:test_main",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "prophnet_feature_extractor_test",
    srcs = ["prophnet_feature_extractor_test.cc"],
    deps = [
        ":prophnet_feature_extractor",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_tnt_feature_extractor_test",
    srcs = ["vehicle_tnt_feature_extractor_test.cc"],
    deps = [
        ":vehicle_tnt_feature",
        ":vehicle_tnt_feature_extractor",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_history_sampler_test",
    srcs = ["object_history_sampler_test.cc"],
    deps = [
        ":object_history_sampler",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
