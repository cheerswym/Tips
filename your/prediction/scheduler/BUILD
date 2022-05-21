load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "priority_analyzer",
    srcs = ["priority_analyzer.cc"],
    hdrs = ["priority_analyzer.h"],
    deps = [
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:prediction_context",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "scenario_analyzer",
    srcs = ["scenario_analyzer.cc"],
    hdrs = ["scenario_analyzer.h"],
    deps = [
        "//onboard/maps:semantic_map_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:prediction_context",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "scheduler",
    srcs = ["scheduler.cc"],
    hdrs = ["scheduler.h"],
    deps = [
        ":priority_analyzer",
        ":scenario_analyzer",
        "//onboard/async:thread_pool",
        "//onboard/lite:logging",
        "//onboard/planner/common:multi_timer_util",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/conflict_resolver",
        "//onboard/prediction/conflict_resolver:conflict_resolver_params",
        "//onboard/prediction/container:model_pool",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/feature_extractor:vehicle_tnt_feature_extractor",
        "//onboard/prediction/long_term:long_term_behavior_estimator",
        "//onboard/prediction/predictor:bicycle_lane_follow_predictor",
        "//onboard/prediction/predictor:heuristic_pedestrian_predictor",
        "//onboard/prediction/predictor:kinematic_predictor",
        "//onboard/prediction/predictor:predictor_util",
        "//onboard/prediction/predictor:prophnet_predictor",
        "//onboard/prediction/predictor:tnt_predictor",
        "//onboard/prediction/predictor:vehicle_lane_follow_predictor",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/types:span",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "scenario_analyzer_test",
    srcs = ["scenario_analyzer_test.cc"],
    deps = [
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction/scheduler:scenario_analyzer",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "priority_analyzer_test",
    srcs = ["priority_analyzer_test.cc"],
    deps = [
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction/scheduler:priority_analyzer",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
