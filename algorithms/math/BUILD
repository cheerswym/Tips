load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//build_tools/build_rules:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//onboard/lite:check",
    ],
)

cc_library(
    name = "eigen",
    hdrs = ["eigen.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "vec",
    srcs = ["vec.cc"],
    hdrs = ["vec.h"],
    deps = [
        ":fast_math",
        "//onboard/math/geometry:affine_transformation_cc_proto",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

cc_library(
    name = "point_types",
    hdrs = ["point_types.h"],
    deps = [":vec"],
)

cc_library(
    name = "stats",
    hdrs = ["stats.h"],
    deps = [
        ":util",
    ],
)

cc_library(
    name = "fast_math",
    hdrs = [
        "fast_math.h",
        "sin_table.h",
    ],
    deps = [
        ":util",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "kalman_filter",
    hdrs = ["kalman_filter.h"],
    deps = [
        ":vec",
        "@eigen",
    ],
)

cc_library(
    name = "hermite_spline",
    hdrs = ["hermite_spline.h"],
)

cc_library(
    name = "cubic_spline",
    srcs = ["cubic_spline.cc"],
    hdrs = ["cubic_spline.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:eigen",
        "//onboard/math:util",
    ],
)

cc_library(
    name = "line_fitter",
    srcs = ["line_fitter.cc"],
    hdrs = ["line_fitter.h"],
    deps = [
        ":eigen",
        ":stats",
        ":util",
        ":vec",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/utils:string_util",
        "@eigen",
    ],
)

cc_library(
    name = "polynomial_fitter",
    srcs = ["polynomial_fitter.cc"],
    hdrs = ["polynomial_fitter.h"],
    deps = [
        ":eigen",
        ":stats",
        ":util",
        ":vec",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/utils:string_util",
        "@eigen",
    ],
)

cc_library(
    name = "estimators",
    srcs = ["estimators.cc"],
    hdrs = ["estimators.h"],
    deps = [
        ":eigen",
        ":vec",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_linear_function",
    hdrs = ["piecewise_linear_function.h"],
    deps = [
        ":piecewise_linear_function_cc_proto",
        ":util",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:util",
    ],
)

cc_library(
    name = "coordinate_converter",
    srcs = ["coordinate_converter.cc"],
    hdrs = ["coordinate_converter.h"],
    deps = [
        "//onboard/global:clock",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/maps:imagery_types",
        "//onboard/maps:map_selector",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/utils:time_util",
    ],
)

cc_library(
    name = "bayesian_world_view",
    srcs = ["bayesian_world_view.cc"],
    hdrs = ["bayesian_world_view.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/proto:bayesian_world_view_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "frenet_frame",
    srcs = ["frenet_frame.cc"],
    hdrs = ["frenet_frame.h"],
    deps = [
        "vec",
        ":util",
        "//onboard/lite:check",
        "//onboard/math/geometry:polygon2d",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "convergence_order",
    srcs = ["convergence_order.cc"],
    hdrs = ["convergence_order.h"],
    deps = [
        "//onboard/math:eigen",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "test_util",
    testonly = True,
    hdrs = ["test_util.h"],
    deps = [
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "@com_google_googletest//:gtest",
    ],
)

###############################################################################
# Protos.
proto_library(
    name = "piecewise_linear_function_proto",
    srcs = ["piecewise_linear_function.proto"],
    deps = [
        "//onboard/math/geometry:affine_transformation_proto",
    ],
)

py_proto_library(
    name = "piecewise_linear_function_py_pb2",
    srcs = ["piecewise_linear_function.proto"],
    deps = [
        "//onboard/math/geometry:affine_transformation_py_pb2",
    ],
)

cc_proto_library(
    name = "piecewise_linear_function_cc_proto",
    deps = [":piecewise_linear_function_proto"],
)

###############################################################################
# Binaries.

cc_binary(
    name = "vec_bm",
    srcs = ["vec_bm.cc"],
    deps = [
        ":vec",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_binary(
    name = "fast_math_bm",
    srcs = ["fast_math_bm.cc"],
    deps = [
        ":fast_math",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "vec_test",
    srcs = ["vec_test.cc"],
    deps = [
        ":vec",
        "//onboard/global:test_main",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/utils:test_util",
    ],
)

cc_test(
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [
        ":util",
        ":vec",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "estimators_test",
    srcs = ["estimators_test.cc"],
    deps = [
        ":estimators",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "piecewise_linear_function_test",
    srcs = ["piecewise_linear_function_test.cc"],
    deps = [
        ":piecewise_linear_function",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "cubic_spline_test",
    srcs = ["cubic_spline_test.cc"],
    deps = [
        ":cubic_spline",
        ":vec",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "line_fitter_test",
    srcs = ["line_fitter_test.cc"],
    deps = [
        ":line_fitter",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "polynomial_fitter_test",
    srcs = ["polynomial_fitter_test.cc"],
    deps = [
        ":polynomial_fitter",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "coordinate_converter_test",
    srcs = ["coordinate_converter_test.cc"],
    deps = [
        ":coordinate_converter",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "frenet_frame_test",
    srcs = ["frenet_frame_test.cc"],
    deps = [
        ":frenet_frame",
        "//onboard/global:test_main",
        "//onboard/lite:check",
    ],
)

cc_test(
    name = "fast_math_test",
    srcs = ["fast_math_test.cc"],
    deps = [
        ":fast_math",
        ":util",
        "//onboard/global:test_main",
    ],
)

cpplint()
