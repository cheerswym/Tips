load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "association_util",
    srcs = ["association_util.cc"],
    hdrs = ["association_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "gate",
    srcs = ["gate.cc"],
    hdrs = ["gate.h"],
    deps = [
        ":association_util",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/perception/multi_camera_fusion:track",
        "//onboard/perception/tracker:track",
        "//onboard/perception/tracker:tracker_util",
        "//onboard/perception/tracker/association/proto:associator_config_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "metric",
    srcs = ["metric.cc"],
    hdrs = ["metric.h"],
    deps = [
        ":association_util",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/perception/multi_camera_fusion:track",
        "//onboard/perception/tracker:track",
        "//onboard/perception/tracker:tracker_util",
        "//onboard/perception/tracker/association/proto:associator_config_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "match_solver",
    srcs = ["match_solver.cc"],
    hdrs = ["match_solver.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:vec",
        "//onboard/perception/tracker/association/proto:associator_config_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "similarity_matrix_builder",
    srcs = ["similarity_matrix_builder.cc"],
    hdrs = ["similarity_matrix_builder.h"],
    deps = [
        ":gate",
        ":metric",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:vec",
        "//onboard/perception/multi_camera_fusion:track",
        "//onboard/perception/tracker:track",
        "//onboard/perception/tracker:tracker_util",
        "//onboard/perception/tracker/association/proto:associator_config_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "onboard_associator",
    srcs = ["onboard_associator.cc"],
    hdrs = ["onboard_associator.h"],
    data = [
        "//onboard/perception/tracker/association/config:association_param",
    ],
    deps = [
        ":match_solver",
        ":similarity_matrix_builder",
        "//onboard/perception/tracker/association/proto:associator_config_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:file_util",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "association_util_test",
    srcs = ["association_util_test.cc"],
    deps = [
        ":association_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "onboard_associator_test",
    srcs = ["onboard_associator_test.cc"],
    data = [
        "//onboard/perception/tracker/association/config:association_param",
    ],
    deps = [
        ":onboard_associator",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "similarity_matrix_builder_test",
    srcs = ["similarity_matrix_builder_test.cc"],
    data = [
        "//onboard/perception/tracker/association/config:association_param",
    ],
    deps = [
        ":similarity_matrix_builder",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "match_solver_test",
    srcs = ["match_solver_test.cc"],
    deps = [
        ":match_solver",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "gate_test",
    srcs = ["gate_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":gate",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "metric_test",
    srcs = ["metric_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":metric",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
