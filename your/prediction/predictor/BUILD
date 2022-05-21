load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "kinematic_predictor",
    srcs = ["kinematic_predictor.cc"],
    hdrs = ["kinematic_predictor.h"],
    deps = [
        "//onboard/maps:semantic_map_manager",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/prediction/util:trajectory_developer",
    ],
)

cc_library(
    name = "heuristic_pedestrian_predictor",
    srcs = ["heuristic_pedestrian_predictor.cc"],
    hdrs = ["heuristic_pedestrian_predictor.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:line_fitter",
        "//onboard/math:stats",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/util:trajectory_developer",
    ],
)

cc_library(
    name = "prophnet_predictor",
    srcs = ["prophnet_predictor.cc"],
    hdrs = ["prophnet_predictor.h"],
    deps = [
        ":predictor_util",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:line_fitter",
        "//onboard/math:polynomial_fitter",
        "//onboard/nets:prophnet_predictor",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/prediction/feature_extractor:prophnet_feature_extractor",
        "//onboard/prediction/predictor:kinematic_predictor",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "bicycle_lane_follow_predictor",
    srcs = ["bicycle_lane_follow_predictor.cc"],
    hdrs = ["bicycle_lane_follow_predictor.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/predictor:kinematic_predictor",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "vehicle_lane_follow_predictor",
    srcs = ["vehicle_lane_follow_predictor.cc"],
    hdrs = ["vehicle_lane_follow_predictor.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/predictor:kinematic_predictor",
        "//onboard/prediction/util:kinematic_model",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/prediction/util:trajectory_developer",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "tnt_predictor",
    srcs = ["tnt_predictor.cc"],
    hdrs = ["tnt_predictor.h"],
    deps = [
        ":predictor_util",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:vec",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:av_context",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/feature_extractor:vehicle_tnt_feature",
        "//onboard/prediction/feature_extractor:vehicle_tnt_feature_extractor",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "predictor_util",
    srcs = ["predictor_util.cc"],
    hdrs = ["predictor_util.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:object_history",
        "//onboard/prediction/container:object_history_span",
        "//onboard/prediction/feature_extractor:feature_extraction_util",
        "//onboard/prediction/feature_extractor:object_history_sampler",
        "//onboard/proto:perception_cc_proto",
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
    name = "heuristic_pedestrian_predictor_test",
    srcs = ["heuristic_pedestrian_predictor_test.cc"],
    deps = [
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/predictor:heuristic_pedestrian_predictor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "bicycle_lane_follow_predictor_test",
    srcs = ["bicycle_lane_follow_predictor_test.cc"],
    deps = [
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/predictor:bicycle_lane_follow_predictor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "vehicle_lane_follow_predictor_test",
    srcs = ["vehicle_lane_follow_predictor_test.cc"],
    deps = [
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/predictor:vehicle_lane_follow_predictor",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "prophnet_predictor_test",
    srcs = ["prophnet_predictor_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/predictor:prophnet_predictor",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
