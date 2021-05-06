load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "freespace_planner",
    srcs = ["freespace_planner.cc"],
    hdrs = ["freespace_planner.h"],
    deps = [
        ":freespace_constraint_builder",
        ":freespace_planner_defs",
        ":path_manager",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:coordinate_converter",
        "//onboard/planner:planner_params",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:planner_util",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/decision:decision_util",
        "//onboard/planner/freespace:tob_path_smoother",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/object:freespace_region_filter",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/speed:speed_finder",
        "//onboard/planner/util:path_util",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "freespace_constraint_builder",
    srcs = ["freespace_constraint_builder.cc"],
    hdrs = ["freespace_constraint_builder.h"],
    deps = [
        ":freespace_planner_defs",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math/geometry:aabox2d",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/freespace:path_manager_util",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "freespace_util",
    srcs = ["freespace_util.cc"],
    hdrs = ["freespace_util.h"],
    deps = [
        "//onboard/maps:maps_common",
        "//onboard/math:coordinate_converter",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/planner/common:global_pose",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "path_manager",
    srcs = ["path_manager.cc"],
    hdrs = ["path_manager.h"],
    deps = [
        ":freespace_planner_defs",
        ":path_manager_util",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/math:coordinate_converter",
        "//onboard/math:util",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:discretized_path",
        "//onboard/planner:planner_params",
        "//onboard/planner/freespace:ipopt_segmented_global_path_smoother",
        "//onboard/planner/freespace/hybrid_a_star",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/utils:proto_util",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "freespace_planner_defs",
    hdrs = ["freespace_planner_defs.h"],
    deps = [
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:segment2d",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
    ],
)

cc_library(
    name = "path_manager_util",
    srcs = ["path_manager_util.cc"],
    hdrs = ["path_manager_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner:discretized_path",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "freespace_planner_plot_util",
    srcs = ["freespace_planner_plot_util.cc"],
    hdrs = ["freespace_planner_plot_util.h"],
    deps = [
        ":path_manager_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math/geometry:aabox2d",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "tob_path_smoother",
    srcs = ["tob_path_smoother.cc"],
    hdrs = ["tob_path_smoother.h"],
    deps = [
        ":freespace_planner_defs",
        ":path_manager_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:trace",
        "//onboard/math:fast_math",
        "//onboard/math:frenet_frame",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math/segment_matcher:segment_matcher_kdtree",
        "//onboard/planner:trajectory_point",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/optimization/ddp:ddp_optimizer",
        "//onboard/planner/optimization/ddp:ddp_optimizer_debug_hook",
        "//onboard/planner/optimization/ddp:object_cost_util",
        "//onboard/planner/optimization/ddp:trajectory_optimizer_util",
        "//onboard/planner/optimization/problem:backward_speed_cost",
        "//onboard/planner/optimization/problem:curvature_cost",
        "//onboard/planner/optimization/problem:forward_speed_cost",
        "//onboard/planner/optimization/problem:intrinsic_jerk_cost",
        "//onboard/planner/optimization/problem:partitioned_object_cost",
        "//onboard/planner/optimization/problem:partitioned_static_boundary_cost",
        "//onboard/planner/optimization/problem:reference_control_deviation_cost",
        "//onboard/planner/optimization/problem:reference_line_deviation_cost",
        "//onboard/planner/optimization/problem:reference_state_deviation_cost",
        "//onboard/planner/optimization/problem:third_order_bicycle",
        "//onboard/planner/optimization/problem:tob_curvature_rate_cost",
        "//onboard/planner/util:path_util",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "ipopt_global_path_smoother",
    srcs = ["ipopt_global_path_smoother.cc"],
    hdrs = ["ipopt_global_path_smoother.h"],
    deps = [
        ":ipopt_global_path_smoother_model",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math:vec",
        "//onboard/planner:discretized_path",
        "//onboard/planner/freespace:path_manager_util",
        "//onboard/planner/optimization/ipopt:ipopt_util",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@ipopt",
    ],
)

cc_library(
    name = "ipopt_global_path_smoother_model",
    srcs = ["ipopt_global_path_smoother_model.cc"],
    hdrs = ["ipopt_global_path_smoother_model.h"],
    deps = [
        "//onboard/global:logging",
        "//onboard/lite:check",
        "//onboard/math:util",
        "//onboard/proto:planner_cc_proto",
        "@adolc",
        "@ipopt",
    ],
)

cc_library(
    name = "ipopt_segmented_global_path_smoother",
    srcs = ["ipopt_segmented_global_path_smoother.cc"],
    hdrs = ["ipopt_segmented_global_path_smoother.h"],
    deps = [
        ":freespace_planner_defs",
        ":ipopt_segmented_global_path_smoother_model",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:trace",
        "//onboard/math:vec",
        "//onboard/planner/freespace:path_manager_util",
        "//onboard/planner/optimization/ipopt:ipopt_util",
        "//onboard/planner/util:path_util",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:vehicle_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@ipopt",
    ],
)

cc_library(
    name = "ipopt_segmented_global_path_smoother_model",
    srcs = ["ipopt_segmented_global_path_smoother_model.cc"],
    hdrs = ["ipopt_segmented_global_path_smoother_model.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/planner/util:path_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "@adolc",
        "@ipopt",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "path_manager_util_test",
    srcs = [
        "path_manager_util_test.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":freespace_planner_plot_util",
        ":path_manager_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_util",
        "//onboard/params:param_manager",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "tob_path_smoother_test",
    srcs = [
        "tob_path_smoother_test.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":tob_path_smoother",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/freespace/hybrid_a_star",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ipopt_global_path_smoother_test",
    srcs = ["ipopt_global_path_smoother_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":ipopt_global_path_smoother",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/freespace/hybrid_a_star",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
