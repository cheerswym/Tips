load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "color",
    srcs = ["color.cc"],
    hdrs = ["color.h"],
    deps = [
        "//onboard/math:eigen",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/vis/common/proto:color_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "colormap",
    srcs = ["colormap.cc"],
    hdrs = ["colormap.h"],
    data = glob([
        "data/*.pb.txt",
    ]),
    deps = [
        ":color",
        "//onboard/lite:check",
        "//onboard/utils:file_util",
        "//onboard/vis/common/proto:color_cc_proto",
        "//onboard/vis/common/proto:colormap_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "obj_loader",
    hdrs = ["obj_loader.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/math:vec",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "renderable_mesh",
    srcs = ["renderable_mesh.cc"],
    hdrs = ["renderable_mesh.h"],
    deps = [
        ":color",
        ":obj_loader",
        "//onboard/math:eigen",
        "//onboard/math:vec",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:triangle_mesh",
        "//onboard/vis/gl:gl_common",
        "//onboard/vis/gl:shader",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "colormap_test",
    srcs = ["colormap_test.cc"],
    deps = [
        ":colormap",
        "//onboard/global:test_main",
    ],
)

cpplint()
