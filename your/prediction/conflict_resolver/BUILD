load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "conflict_resolver",
    srcs = ["conflict_resolver.cc"],
    hdrs = ["conflict_resolver.h"],
    deps = [
        ":conflict_resolver_input",
        ":conflict_resolver_params",
        ":conflict_resolver_util",
        ":constraint_builder",
        ":object_conflict_manager",
        ":object_conflict_resolver",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/container:object_prediction_result",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/util:lane_path_finder",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/utils:file_util",
    ],
)

cc_library(
    name = "object_conflict_manager",
    srcs = ["object_conflict_manager.cc"],
    hdrs = ["object_conflict_manager.h"],
    deps = [
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/prediction/container:object_prediction_result",
        "//onboard/prediction/container:prediction_context",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "conflict_resolver_input",
    hdrs = ["conflict_resolver_input.h"],
    deps = [
        ":conflict_resolver_params",
        ":object_conflict_manager",
        "//onboard/planner:discretized_path",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction/container:object_prediction_result",
        "//onboard/prediction/container:prediction_context",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "constraint_builder",
    srcs = ["constraint_builder.cc"],
    hdrs = ["constraint_builder.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/prediction/container:traffic_light_manager",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "object_svt_sample",
    srcs = ["object_svt_sample.cc"],
    hdrs = ["object_svt_sample.h"],
    deps = [
        "//onboard/container:strong_vector",
        "//onboard/math:util",
        "//onboard/planner:planner_defs",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "object_st_map",
    srcs = ["object_st_map.cc"],
    hdrs = ["object_st_map.h"],
    deps = [
        ":conflict_resolver_params",
        ":object_conflict_manager",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/maps:maps_common",
        "//onboard/maps:maps_helper",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:discretized_path",
        "//onboard/planner/speed:speed_vector",
        "//onboard/planner/speed:st_boundary",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "svt_feature_cost",
    srcs = ["svt_feature_cost.cc"],
    hdrs = ["svt_feature_cost.h"],
    deps = [
        ":object_svt_sample",
        "//onboard/planner/speed:speed_vector",
        "//onboard/planner/speed:st_boundary",
        "//onboard/proto:prediction_cc_proto",
    ],
)

cc_library(
    name = "svt_cost_provider",
    srcs = ["svt_cost_provider.cc"],
    hdrs = ["svt_cost_provider.h"],
    deps = [
        ":object_st_map",
        ":object_svt_sample",
        ":svt_feature_cost",
        "//onboard/lite:check",
        "//onboard/planner/speed:st_boundary",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "conflict_resolver_params",
    srcs = ["conflict_resolver_params.cc"],
    hdrs = ["conflict_resolver_params.h"],
    data = [
        ":default_params.pb.txt",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/utils:file_util",
    ],
)

cc_library(
    name = "conflict_resolver_util",
    srcs = ["conflict_resolver_util.cc"],
    hdrs = ["conflict_resolver_util.h"],
    deps = [
        ":object_svt_sample",
        "//onboard/maps:semantic_map_manager",
        "//onboard/planner:discretized_path",
        "//onboard/planner/speed:speed_vector",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "object_svt_graph",
    srcs = ["object_svt_graph.cc"],
    hdrs = ["object_svt_graph.h"],
    deps = [
        ":conflict_resolver_params",
        ":conflict_resolver_util",
        ":object_svt_sample",
        ":svt_cost_provider",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/prediction:prediction_flags",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "object_conflict_resolver",
    srcs = ["object_conflict_resolver.cc"],
    hdrs = ["object_conflict_resolver.h"],
    deps = [
        ":conflict_resolver_input",
        ":object_conflict_manager",
        ":object_st_map",
        ":object_svt_graph",
        "//onboard/async:thread_pool",
        "//onboard/planner:discretized_path",
        "//onboard/prediction:prediction_flags",
        "//onboard/prediction/util:trajectory_developer",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "relation_analyzer",
    srcs = ["relation_analyzer.cc"],
    hdrs = ["relation_analyzer.h"],
    deps = [
        "//onboard/math/geometry:box2d",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/status:statusor",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "object_svt_sample_test",
    srcs = ["object_svt_sample_test.cc"],
    deps = [
        ":object_svt_sample",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "svt_feature_cost_test",
    srcs = ["svt_feature_cost_test.cc"],
    deps = [
        ":conflict_resolver_params",
        ":object_svt_sample",
        ":svt_feature_cost",
        "//onboard/global:test_main",
        "//onboard/planner/speed:speed_point",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "object_svt_graph_test",
    srcs = ["object_svt_graph_test.cc"],
    deps = [
        ":conflict_resolver_params",
        ":conflict_resolver_util",
        ":object_svt_graph",
        ":object_svt_sample",
        ":svt_cost_provider",
        "//onboard/global:test_main",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
