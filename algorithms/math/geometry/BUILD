load("@com_google_protobuf//:protobuf.bzl", "py_proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("//build_tools/build_rules:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "affine_transformation",
    srcs = ["affine_transformation.cc"],
    hdrs = ["affine_transformation.h"],
    deps = [
        ":affine_transformation_cc_proto",
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "triangle_mesh",
    srcs = ["triangle_mesh.cc"],
    hdrs = ["triangle_mesh.h"],
    deps = [
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "kdtree",
    srcs = [
        "kdtree.cc",
        "kdtree.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:point_types",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "rtree",
    hdrs = ["rtree.h"],
    deps = [
        "//onboard/maps/proto:geometry_cc_proto",
        "//onboard/math:vec",
        "@boost//:geometry",
    ],
)

cc_library(
    name = "segment2d",
    srcs = [
        "segment2d.cc",
        "segment2d.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "aabox2d",
    srcs = [
        "aabox2d.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "box2d",
    srcs = [
        "box2d.cc",
        "box2d.h",
    ],
    deps = [
        ":aabox2d",
        ":box2d_cc_proto",
        ":segment2d",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "polygon2d",
    srcs = [
        "polygon2d.cc",
        "polygon2d.h",
    ],
    deps = [
        ":box2d",
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "common_shapes",
    hdrs = ["common_shapes.h"],
    deps = [
        ":aabox2d",
        ":box2d",
        ":polygon2d",
        ":segment2d",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "line_intersection",
    srcs = ["line_intersection.cc"],
    hdrs = ["line_intersection.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:vec",
        "//onboard/math/geometry:polyline2d",
    ],
)

cc_library(
    name = "polyline2d",
    srcs = ["polyline2d.cc"],
    hdrs = ["polyline2d.h"],
    deps = [
        ":common_shapes",
        "//onboard/lite:check",
        "//onboard/math:util",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:check",
        "//onboard/maps/proto:geometry_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:polygon2d",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "pose_interpolation",
    srcs = ["pose_interpolation.cc"],
    hdrs = ["pose_interpolation.h"],
    deps = [
        ":util",
        "//onboard/math:piecewise_linear_function",
    ],
)

cc_library(
    name = "aabox_kdtree2d",
    srcs = ["aabox_kdtree2d.h"],
    deps = [
        ":aabox2d",
        "//onboard/math:util",
    ],
)

cc_library(
    name = "halfplane",
    hdrs = ["halfplane.h"],
    deps = [
        ":halfplane_cc_proto",
        ":util",
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "circle2d",
    hdrs = ["circle2d.h"],
    deps = [
        "//onboard/math:vec",
    ],
)

cc_library(
    name = "bezier_curve",
    srcs = ["bezier_curve.cc"],
    hdrs = ["bezier_curve.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "ellipse",
    hdrs = ["ellipse.h"],
    deps = [],
)

###############################################################################
# Protos.
proto_library(
    name = "affine_transformation_proto",
    srcs = ["affine_transformation.proto"],
)

py_proto_library(
    name = "affine_transformation_py_pb2",
    srcs = ["affine_transformation.proto"],
    deps = ["@com_google_protobuf//:protobuf_python"],
)

cc_proto_library(
    name = "affine_transformation_cc_proto",
    deps = [":affine_transformation_proto"],
)

proto_library(
    name = "halfplane_proto",
    srcs = ["halfplane.proto"],
    deps = [
        ":affine_transformation_proto",
    ],
)

py_proto_library(
    name = "halfplane_py_pb2",
    srcs = ["halfplane.proto"],
    deps = [
        "affine_transformation_py_pb2",
        "@com_google_protobuf//:protobuf_python",
    ],
)

cc_proto_library(
    name = "halfplane_cc_proto",
    deps = [":halfplane_proto"],
)

proto_library(
    name = "box2d_proto",
    srcs = ["box2d.proto"],
)

py_proto_library(
    name = "box2d_py_pb2",
    srcs = ["box2d.proto"],
    deps = ["@com_google_protobuf//:protobuf_python"],
)

cc_proto_library(
    name = "box2d_cc_proto",
    deps = [":box2d_proto"],
)

proto_library(
    name = "aabox3d_proto",
    srcs = ["aabox3d.proto"],
)

cc_proto_library(
    name = "aabox3d_cc_proto",
    deps = [":aabox3d_proto"],
)

py_proto_library(
    name = "aabox3d_py_pb2",
    srcs = ["aabox3d.proto"],
    deps = ["@com_google_protobuf//:protobuf_python"],
)

###############################################################################
# Binaries.
cc_binary(
    name = "line_intersection_bm",
    srcs = ["line_intersection_bm.cc"],
    deps = [
        ":line_intersection",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_binary(
    name = "aabox2d_bm",
    srcs = ["aabox2d_bm.cc"],
    deps = [
        ":aabox2d",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_binary(
    name = "polygon2d_bm",
    srcs = ["polygon2d_bm.cc"],
    deps = [
        ":polygon2d",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_binary(
    name = "box2d_bm",
    srcs = ["box2d_bm.cc"],
    deps = [
        ":aabox2d",
        ":box2d",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_binary(
    name = "aabox_kdtree2d_bm",
    srcs = ["aabox_kdtree2d_bm.cc"],
    deps = [
        ":aabox_kdtree2d",
        "@com_github_google_benchmark//:benchmark",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "affine_transformation_test",
    srcs = ["affine_transformation_test.cc"],
    deps = [
        ":affine_transformation",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "triangle_mesh_test",
    srcs = ["triangle_mesh_test.cc"],
    deps = [
        ":triangle_mesh",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "kdtree_test",
    srcs = [
        "kdtree_test.cc",
    ],
    deps = [
        ":kdtree",
        "//onboard/global:test_main",
        "//onboard/lite:check",
    ],
)

cc_test(
    name = "segment2d_test",
    srcs = [
        "segment2d_test.cc",
    ],
    deps = [
        ":segment2d",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
    ],
)

cc_test(
    name = "aabox2d_test",
    srcs = [
        "aabox2d_test.cc",
    ],
    deps = [
        ":aabox2d",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
    ],
)

cc_test(
    name = "box2d_test",
    srcs = [
        "box2d_test.cc",
    ],
    deps = [
        ":box2d",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "//onboard/math/geometry:polygon2d",
    ],
)

cc_test(
    name = "polygon2d_test",
    srcs = [
        "polygon2d_test.cc",
    ],
    deps = [
        ":polygon2d",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
    ],
)

cc_test(
    name = "pose_interpolation_test",
    srcs = [
        "pose_interpolation_test.cc",
    ],
    deps = [
        ":pose_interpolation",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "line_intersection_test",
    srcs = [
        "line_intersection_test.cc",
    ],
    deps = [
        ":line_intersection",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "aabox_kdtree2d_test",
    srcs = [
        "aabox_kdtree2d_test.cc",
    ],
    deps = [
        ":aabox2d",
        ":aabox_kdtree2d",
        ":segment2d",
        "//onboard/global:test_main",
    ],
)

cc_test(
    name = "halfplane_test",
    srcs = [
        "halfplane_test.cc",
    ],
    deps = [
        ":halfplane",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
    ],
)

cc_test(
    name = "circle2d_test",
    srcs = ["circle2d_test.cc"],
    deps = [
        ":circle2d",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
    ],
)

cc_test(
    name = "bezier_curve_test",
    srcs = ["bezier_curve_test.cc"],
    deps = [
        ":bezier_curve",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
    ],
)

cc_test(
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [
        ":util",
        "//onboard/global:test_main",
        "//onboard/math:util",
    ],
)

cpplint()
