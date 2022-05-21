load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacle_builder",
    testonly = True,
    srcs = ["obstacle_builder.cc"],
    hdrs = ["obstacle_builder.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:polygon2d",
        "//onboard/perception:obstacle",
    ],
)

cc_library(
    name = "cluster_builder",
    testonly = True,
    srcs = ["cluster_builder.cc"],
    hdrs = ["cluster_builder.h"],
    deps = [
        ":obstacle_builder",
        "//onboard/perception:cluster",
    ],
)

cc_library(
    name = "measurement_builder",
    testonly = True,
    srcs = ["measurement_builder.cc"],
    hdrs = ["measurement_builder.h"],
    deps = [
        "//onboard/math:test_util",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:radar_cc_proto",
    ],
)

cc_library(
    name = "track_builder",
    testonly = True,
    srcs = ["track_builder.cc"],
    hdrs = ["track_builder.h"],
    deps = [
        "//onboard/math:test_util",
        "//onboard/math/geometry:util",
        "//onboard/perception/tracker:track",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_test(
    name = "obstacle_builder_test",
    srcs = ["obstacle_builder_test.cc"],
    deps = [
        ":obstacle_builder",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "cluster_builder_test",
    srcs = ["cluster_builder_test.cc"],
    deps = [
        ":cluster_builder",
        ":obstacle_builder",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "measurement_builder_test",
    srcs = ["measurement_builder_test.cc"],
    deps = [
        ":measurement_builder",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "track_builder_test",
    srcs = ["track_builder_test.cc"],
    deps = [
        ":measurement_builder",
        ":track_builder",
        "//onboard/global:test_main",
        "//onboard/utils:test_util",
    ],
)

cpplint()
