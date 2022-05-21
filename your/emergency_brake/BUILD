load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])
###############################################################################
# Libraries.

cc_library(
    name = "emergency_brake_module",
    srcs = ["emergency_brake_module.cc"],
    hdrs = ["emergency_brake_module.h"],
    deps = [
        ":util",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/math:safe_unit",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:planner_params",
        "//onboard/planner:planner_util",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:emergency_brake_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//onboard/math:safe_unit",
        "//onboard/math:util",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
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
    name = "util_test",
    srcs = ["util_test.cc"],
    deps = [
        ":util",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:util",
        "//onboard/utils:status_macros",
        "//onboard/utils:test_util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
