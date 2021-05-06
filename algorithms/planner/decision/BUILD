load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "decider_input",
    hdrs = ["decider_input.h"],
    deps = [
        "//onboard/params/v2/proto/vehicle:common_cc_proto",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_library(
    name = "decider_output",
    hdrs = ["decider_output.h"],
    deps = [
        ":constraint_manager",
        "//onboard/planner/decision/proto:crosswalk_state_cc_proto",
    ],
)

cc_library(
    name = "precedence_decider",
    srcs = ["precedence_decider.cc"],
    hdrs = ["precedence_decider.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/math:util",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:smooth_lane_path",
        "//onboard/prediction",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "tl_info",
    srcs = ["tl_info.cc"],
    hdrs = ["tl_info.h"],
    deps = [
        "//onboard/maps:semantic_map_defs",
        "//onboard/planner/decision/proto:traffic_light_info_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "traffic_light_info_collector",
    srcs = ["traffic_light_info_collector.cc"],
    hdrs = ["traffic_light_info_collector.h"],
    deps = [
        ":tl_info",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner/router:route_sections",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:time_util",
    ],
)

cc_library(
    name = "traffic_light_decider",
    srcs = ["traffic_light_decider.cc"],
    hdrs = ["traffic_light_decider.h"],
    deps = [
        "//onboard/planner:planner_params",
        "//onboard/planner:speed_profile",
        "//onboard/planner/decision:decider_output",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "pedestrians_decider",
    srcs = ["pedestrians_decider.cc"],
    hdrs = ["pedestrians_decider.h"],
    deps = [
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "turn_signal_decider",
    srcs = ["turn_signal_decider.cc"],
    hdrs = ["turn_signal_decider.h"],
    deps = [
        "//onboard/planner:planner_defs",
        "//onboard/planner:teleop_state",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
    ],
)

cc_library(
    name = "door_open_decider",
    srcs = ["door_open_decider.cc"],
    hdrs = ["door_open_decider.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/planner:planner_flags",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "stop_sign_decider",
    srcs = ["stop_sign_decider.cc"],
    hdrs = ["stop_sign_decider.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/decision/proto:stop_sign_state_cc_proto",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/util:perception_util",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "crosswalk_decider",
    srcs = ["crosswalk_decider.cc"],
    hdrs = ["crosswalk_decider.h"],
    deps = [
        ":decider_output",
        "//onboard/eval:qevent",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "constraint_manager",
    hdrs = ["constraint_manager.h"],
    deps = [
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:halfplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "constraint_builder",
    srcs = ["constraint_builder.cc"],
    hdrs = ["constraint_builder.h"],
    deps = [
        ":beyond_length_along_route",
        ":cautious_brake_decider",
        ":crosswalk_decider",
        ":decider_input",
        ":decider_output",
        ":decision_util",
        ":end_of_current_lane_path",
        ":end_of_path_boundary",
        ":ignore_object",
        ":lc_end_of_current_lane_constraint",
        ":leading_object",
        ":no_block",
        ":parking_brake_release",
        ":pedestrians_decider",
        ":speed_bump",
        ":standstill_decider",
        ":stop_sign_decider",
        ":toll_decider",
        ":traffic_light_decider",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math/geometry:halfplane",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner:planner_flags",
        "//onboard/planner:planner_params",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "leading_object",
    srcs = ["leading_object.cc"],
    hdrs = ["leading_object.h"],
    deps = [
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_flags",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/container:flat_hash_set",
    ],
)

cc_library(
    name = "ignore_object",
    srcs = ["ignore_object.cc"],
    hdrs = ["ignore_object.h"],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "no_block",
    srcs = ["no_block.cc"],
    hdrs = ["no_block.h"],
    deps = [
        ":decision_util",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "cautious_brake_decider",
    srcs = ["cautious_brake_decider.cc"],
    hdrs = ["cautious_brake_decider.h"],
    deps = [
        ":decision_util",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner:planner_defs",
        "//onboard/planner:planner_util",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "end_of_path_boundary",
    srcs = ["end_of_path_boundary.cc"],
    hdrs = ["end_of_path_boundary.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "end_of_current_lane_path",
    srcs = ["end_of_current_lane_path.cc"],
    hdrs = ["end_of_current_lane_path.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "speed_bump",
    srcs = ["speed_bump.cc"],
    hdrs = ["speed_bump.h"],
    deps = [
        "//onboard/maps:maps_helper",
        "//onboard/maps:semantic_map_util",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "decision_util",
    srcs = ["decision_util.cc"],
    hdrs = ["decision_util.h"],
    deps = [
        "//onboard/global:trace",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner:planner_defs",
        "//onboard/planner:speed_profile",
        "//onboard/planner/common:path_sl_boundary",
        "//onboard/planner/decision:constraint_manager",
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "lc_end_of_current_lane_constraint",
    srcs = ["lc_end_of_current_lane_constraint.cc"],
    hdrs = ["lc_end_of_current_lane_constraint.h"],
    deps = [
        "//onboard/maps:lane_path",
        "//onboard/planner:planner_defs",
        "//onboard/planner/router:drive_passage",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:status_macros",
    ],
)

cc_library(
    name = "beyond_length_along_route",
    srcs = ["beyond_length_along_route.cc"],
    hdrs = ["beyond_length_along_route.h"],
    deps = [
        "//onboard/planner:planner_defs",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "parking_brake_release",
    srcs = ["parking_brake_release.cc"],
    hdrs = ["parking_brake_release.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/router:drive_passage",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "toll_decider",
    srcs = ["toll_decider.cc"],
    hdrs = ["toll_decider.h"],
    deps = [
        "//onboard/planner/router:drive_passage",
    ],
)

cc_library(
    name = "standstill_decider",
    srcs = ["standstill_decider.cc"],
    hdrs = ["standstill_decider.h"],
    deps = [
        "//onboard/math/geometry:halfplane",
        "//onboard/planner/router:drive_passage",
        "//onboard/utils:status_macros",
    ],
)

###############################################################################
# Protos.

###############################################################################
# Binaries.

###############################################################################
# Tests.

cc_test(
    name = "constraint_builder_test",
    srcs = ["constraint_builder_test.cc"],
    deps = [
        ":constraint_builder",
        "//onboard/global:test_main",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "leading_object_test",
    srcs = ["leading_object_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":leading_object",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_util",
        "//onboard/params:param_manager",
        "//onboard/planner/initializer:test_util",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/util:vehicle_geometry_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "ignore_object_test",
    srcs = ["ignore_object_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":ignore_object",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:planner_object_builder",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "no_block_test",
    srcs = ["no_block_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":no_block",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "cautious_brake_decider_test",
    srcs = ["cautious_brake_decider_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":cautious_brake_decider",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "end_of_path_boundary_test",
    srcs = ["end_of_path_boundary_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":end_of_path_boundary",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/scheduler:path_boundary_builder",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "//onboard/utils:status_macros",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "end_of_current_lane_path_test",
    srcs = ["end_of_current_lane_path_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":end_of_current_lane_path",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_bump_test",
    srcs = ["speed_bump_test.cc"],
    deps = [
        ":speed_bump",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "decision_util_test",
    srcs = ["decision_util_test.cc"],
    deps = [
        ":decision_util",
        "//onboard/global:test_main",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "parking_brake_release_test",
    srcs = ["parking_brake_release_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":parking_brake_release",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:clock",
        "//onboard/global:test_main",
        "//onboard/params:param_manager",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "traffic_light_info_collector_test",
    srcs = ["traffic_light_info_collector_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":traffic_light_info_collector",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/test_util:route_builder",
        "//onboard/planner/test_util:util",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
