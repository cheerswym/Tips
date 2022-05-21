load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "prediction_object",
    srcs = ["prediction_object.cc"],
    hdrs = ["prediction_object.h"],
    deps = [
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/prediction:prediction_defs",
        "//onboard/prediction/util:transform_util",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "object_history_span",
    srcs = ["object_history_span.cc"],
    hdrs = ["object_history_span.h"],
    deps = [
        ":prediction_object",
        "//onboard/prediction:prediction_defs",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "object_history",
    srcs = ["object_history.cc"],
    hdrs = ["object_history.h"],
    deps = [
        ":object_history_span",
        ":prediction_object",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "objects_history",
    srcs = ["objects_history.cc"],
    hdrs = ["objects_history.h"],
    deps = [
        ":object_history",
        ":object_history_span",
        ":prediction_object",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/math:coordinate_converter",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:localization_cc_proto",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "object_prediction_result",
    hdrs = ["object_prediction_result.h"],
    deps = [
        "//onboard/prediction:predicted_trajectory",
        "//onboard/prediction/container:object_history",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "object_prediction_history_span",
    srcs = ["object_prediction_history_span.cc"],
    hdrs = ["object_prediction_history_span.h"],
    deps = [
        ":object_prediction_result",
        "//onboard/prediction:prediction_defs",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "object_predictions_history",
    srcs = ["object_predictions_history.cc"],
    hdrs = ["object_predictions_history.h"],
    deps = [
        ":object_prediction_history_span",
        ":object_prediction_result",
        "//onboard/global:clock",
        "//onboard/global:timer",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "av_context",
    srcs = ["av_context.cc"],
    hdrs = ["av_context.h"],
    deps = [
        ":objects_history",
        ":prediction_object",
        "//onboard/planner/util:perception_util",
        "//onboard/prediction:prediction_defs",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:elements_history",
    ],
)

cc_library(
    name = "prediction_input",
    srcs = ["prediction_input.cc"],
    hdrs = ["prediction_input.h"],
    deps = [
        ":av_context",
        ":objects_history",
        ":prediction_state",
        "//onboard/global:clock",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/prediction/conflict_resolver:conflict_resolver_params",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:proto_util",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "prediction_state",
    srcs = ["prediction_state.cc"],
    hdrs = ["prediction_state.h"],
    deps = [
        ":av_context",
        ":objects_history",
        "//onboard/global:clock",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:proto_util",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "prediction_runner",
    srcs = ["prediction_runner.cc"],
    hdrs = ["prediction_runner.h"],
    deps = [
        ":model_pool",
        ":prediction_context",
        ":prediction_input",
        "//onboard/async:thread_pool",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_main_loop_internal",  #GetAllObjects
        "//onboard/prediction/scheduler",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "prediction_context",
    srcs = ["prediction_context.cc"],
    hdrs = ["prediction_context.h"],
    deps = [
        ":av_context",
        ":object_history_span",
        ":object_prediction_history_span",
        ":object_predictions_history",
        ":objects_history",
        ":prediction_input",
        ":prediction_object",
        ":traffic_light_manager",
        "//onboard/async:thread_pool",
        "//onboard/global:clock",
        "//onboard/maps:semantic_map_manager",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "traffic_light_manager",
    srcs = ["traffic_light_manager.cc"],
    hdrs = ["traffic_light_manager.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "model_pool",
    srcs = ["model_pool.cc"],
    hdrs = ["model_pool.h"],
    deps = [
        "//onboard/nets:prophnet_predictor",
        "//onboard/params:param_manager",
        "//onboard/prediction:prediction_flags",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "objects_history_test",
    srcs = ["objects_history_test.cc"],
    deps = [
        ":objects_history",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "prediction_state_test",
    srcs = ["prediction_state_test.cc"],
    deps = [
        ":prediction_state",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:proto_util",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
