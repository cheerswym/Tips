load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "planner_module",
    srcs = ["planner_module.cc"],
    hdrs = ["planner_module.h"],
    deps = [
        ":manual_trajectory_util",
        ":planner_defs",
        ":planner_flags",
        ":planner_input",
        ":planner_main_loop_internal",
        ":planner_params",
        ":planner_semantic_map_manager",
        ":planner_sim_flags",
        ":planner_state",
        ":planner_util",
        ":standby_state",
        ":teleop_state",
        "//common/proto:semantic_map_modifier_cc_proto",
        "//offboard/vis/ark/ark_server:ark_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_util",
        "//onboard/async:future",
        "//onboard/async:thread_pool",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:buffered_logger",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:logging",
        "//onboard/global:timer",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:lite_client_base",
        "//onboard/lite:lite_module",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_io",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:piecewise_linear_function",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/common/proto:planner_status_cc_proto",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/plan:plan_task_dispatcher",
        "//onboard/planner/proto:planner_output_cc_proto",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router:route_manager_output",
        "//onboard/planner/router:route_recorder_module",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/prediction:prediction_util",
        "//onboard/prediction/conflict_resolver:conflict_resolver_params",
        "//onboard/prediction/container:model_pool",
        "//onboard/prediction/container:prediction_context",
        "//onboard/prediction/container:prediction_input",
        "//onboard/prediction/container:prediction_runner",
        "//onboard/prediction/container:prediction_state",
        "//onboard/prediction/scheduler",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:localization_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:q_issue_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:semantic_map_modification_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/cleanup",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "planner_main_loop_internal",
    srcs = ["planner_main_loop_internal.cc"],
    hdrs = ["planner_main_loop_internal.h"],
    deps = [
        ":planner_defs",
        ":planner_util",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/eval:qevent",
        "//onboard/eval:qevent_base",
        "//onboard/global:buffered_logger",
        "//onboard/global:logging",
        "//onboard/global:trace",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:maps_common",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry/proto:affine_transformation_cc_proto",
        "//onboard/planner:planner_semantic_map_manager",
        "//onboard/planner:planner_state",
        "//onboard/planner:teleop_state",
        "//onboard/planner:trajectory_util",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/decision/proto:traffic_light_info_cc_proto",
        "//onboard/planner/plan:plan_task",
        "//onboard/planner/plan:plan_task_helper",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/proto:trajectory_validation_cc_proto",
        "//onboard/planner/router:route",
        "//onboard/planner/router:route_manager_output",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/router:route_sections_info",
        "//onboard/planner/router:route_sections_util",
        "//onboard/planner/router:route_util",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/planner/scheduler:smooth_reference_line_builder",
        "//onboard/planner/scheduler:smooth_reference_line_result",
        "//onboard/planner/scheduler/proto:lane_change_cc_proto",
        "//onboard/planner/scheduler/proto:smooth_reference_line_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:trajectory_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/proto:turn_signal_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:objects_view",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "trajectory_util",
    srcs = ["trajectory_util.cc"],
    hdrs = ["trajectory_util.h"],
    deps = [
        ":second_order_trajectory_point",
        ":trajectory_point",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/planner/util:path_util",
        "//onboard/prediction:predicted_trajectory",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_validation",
    srcs = ["trajectory_validation.cc"],
    hdrs = ["trajectory_validation.h"],
    deps = [
        ":planner_defs",
        ":planner_util",
        ":trajectory_point",
        ":trajectory_util",
        ":trajectory_validation_error",
        "//onboard/async:parallel_for",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/math:frenet_frame",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/util:vehicle_geometry_util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "trajectory_validation_error",
    hdrs = [
        "trajectory_validation_error.h",
    ],
    deps = [
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "second_order_trajectory_point",
    srcs = ["second_order_trajectory_point.cc"],
    hdrs = ["second_order_trajectory_point.h"],
    deps = [
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "trajectory_point",
    srcs = ["trajectory_point.cc"],
    hdrs = ["trajectory_point.h"],
    deps = [
        ":second_order_trajectory_point",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "smooth_lane",
    srcs = ["smooth_lane.cc"],
    hdrs = ["smooth_lane.h"],
    deps = [
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/utils:file_util",
    ],
)

cc_library(
    name = "smooth_lane_path",
    srcs = ["smooth_lane_path.cc"],
    hdrs = ["smooth_lane_path.h"],
    deps = [
        ":smooth_lane",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:coordinate_converter",
        "//onboard/math:util",
        "//onboard/utils:file_util",
        "//onboard/vis/common:colormap",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "discretized_path",
    srcs = ["discretized_path.cc"],
    hdrs = ["discretized_path.h"],
    deps = [
        ":trajectory_util",
        "//onboard/lite:logging",
        "//onboard/math:frenet_common",
        "//onboard/math:vec",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "speed_profile",
    hdrs = ["speed_profile.h"],
    deps = [
        "//onboard/math:piecewise_linear_function",
    ],
)

cc_library(
    name = "control_error_model",
    hdrs = ["control_error_model.h"],
    deps = [
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:util",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:positioning_cc_proto",
    ],
)

cc_library(
    name = "planner_util",
    srcs = ["planner_util.cc"],
    hdrs = ["planner_util.h"],
    deps = [
        ":planner_defs",
        ":planner_flags",
        ":planner_semantic_map_manager",
        ":trajectory_point",
        ":trajectory_util",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps:semantic_map_util",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/router:route_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "mfob_trajectory_smoother",
    srcs = ["mfob_trajectory_smoother.cc"],
    hdrs = ["mfob_trajectory_smoother.h"],
    deps = [
        ":planner_params",
        ":trajectory_point",
        "//onboard/eval:qevent",
        "//onboard/lite:logging",
        "//onboard/math:piecewise_linear_function",
        "//onboard/planner/optimization/ddp:ddp_optimizer",
        "//onboard/planner/optimization/problem:curvature_cost",
        "//onboard/planner/optimization/problem:forward_speed_cost",
        "//onboard/planner/optimization/problem:longitudinal_acceleration_cost",
        "//onboard/planner/optimization/problem:mfob_curvature_rate_cost",
        "//onboard/planner/optimization/problem:mixed_fourth_order_bicycle",
        "//onboard/planner/optimization/problem:reference_control_deviation_cost",
        "//onboard/planner/optimization/problem:reference_line_deviation_cost",
        "//onboard/planner/optimization/problem:reference_state_deviation_cost",
        "//onboard/planner/util:trajectory_plot_util",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "planner_defs",
    hdrs = ["planner_defs.h"],
    deps = [
        "//onboard/math:util",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "planner_flags",
    srcs = ["planner_flags.cc"],
    hdrs = ["planner_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "planner_sim_flags",
    srcs = ["planner_sim_flags.cc"],
    hdrs = ["planner_sim_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "composite_lane_path",
    srcs = ["composite_lane_path.cc"],
    hdrs = ["composite_lane_path.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/maps:lane_point",
        "//onboard/maps:semantic_map_defs",
        "//onboard/maps:semantic_map_manager",
        "//onboard/maps/proto:lane_path_cc_proto",
        "//onboard/maps/proto:semantic_map_cc_proto",
        "//onboard/math:util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "planner_params",
    srcs = ["planner_params.cc"],
    hdrs = ["planner_params.h"],
    data = [
        "//onboard/planner/params:param_files",
    ],
    deps = [
        "//onboard/base",
        "//onboard/global:car_common",
        "//onboard/global:singleton",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/maps:map_selector",
        "//onboard/params:param_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/utils:proto_util",
    ],
)

cc_library(
    name = "emergency_stop",
    srcs = ["emergency_stop.cc"],
    hdrs = ["emergency_stop.h"],
    deps = [
        ":planner_defs",
        ":planner_input",
        ":planner_params",
        ":trajectory_point",
        ":trajectory_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/math:frenet_frame",
        "//onboard/math:util",
        "//onboard/math/geometry:polygon2d",
        "//onboard/planner/object:planner_object",
        "//onboard/planner/proto:planner_params_cc_proto",
        "//onboard/planner/util:path_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:status_macros",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "standby_state",
    srcs = ["standby_state.cc"],
    hdrs = ["standby_state.h"],
    deps = [
        ":planner_defs",
        ":trajectory_point",
        "//onboard/math/geometry:halfplane",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/proto:trajectory_point_cc_proto",
        "//onboard/utils:status_macros",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "planner_input",
    hdrs = ["planner_input.h"],
    deps = [
        ":planner_params",
        ":planner_state",
        "//onboard/async:async_util",
        "//onboard/maps:semantic_map_manager",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/router/proto:route_manager_output_cc_proto",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:chassis_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/proto:prediction_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/proto:route_cc_proto",
        "//onboard/proto:semantic_map_modification_cc_proto",
        "//onboard/utils:objects_view",
    ],
)

cc_library(
    name = "planner_state",
    srcs = ["planner_state.cc"],
    hdrs = ["planner_state.h"],
    deps = [
        ":planner_semantic_map_manager",
        ":planner_util",
        "//onboard/global:trace",
        "//onboard/maps:semantic_map_defs",
        "//onboard/math/geometry:util",
        "//onboard/planner/decision:traffic_light_info_collector",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/plan:plan_task",
        "//onboard/planner/proto:planner_state_cc_proto",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/scheduler:smooth_reference_line_builder",
        "//onboard/prediction/container:prediction_state",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:planner_cc_proto",
        "//onboard/utils:proto_util",
        "//onboard/utils:time_util",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "manual_trajectory_util",
    srcs = ["manual_trajectory_util.cc"],
    hdrs = ["manual_trajectory_util.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:coordinate_converter",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "est_planner",
    srcs = ["est_planner.cc"],
    hdrs = ["est_planner.h"],
    data = [
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning:dopt_auto_tuning_files",
    ],
    deps = [
        ":est_planner_output",
        ":min_length_path_extension",
        ":planner_defs",
        ":planner_flags",
        ":planner_main_loop_internal",
        ":planner_state",
        ":teleop_state",
        ":trajectory_point",
        ":trajectory_util",
        ":trajectory_validation",
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning:auto_tuning_common_flags",
        "//offboard/vis/vantage/charts:chart_util",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:async_util",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:util",
        "//onboard/params:vehicle_param_api",
        "//onboard/planner/common:multi_timer_util",
        "//onboard/planner/common:plan_start_point_info",
        "//onboard/planner/common:planner_status",
        "//onboard/planner/decision:constraint_builder",
        "//onboard/planner/decision:decider_input",
        "//onboard/planner/decision:decision_util",
        "//onboard/planner/decision:tl_info",
        "//onboard/planner/initializer:initializer_input",
        "//onboard/planner/initializer:initializer_output",
        "//onboard/planner/initializer:initializer_util",
        "//onboard/planner/initializer:search_motion",
        "//onboard/planner/object:planner_object_manager",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/object:spacetime_trajectory_manager_builder",
        "//onboard/planner/optimization/ddp:trajectory_optimizer",
        "//onboard/planner/optimization/ddp:trajectory_optimizer_input",
        "//onboard/planner/optimization/ddp:trajectory_optimizer_output",
        "//onboard/planner/optimization/ddp:trajectory_optimizer_util",
        "//onboard/planner/plan:st_path_planner",
        "//onboard/planner/router:drive_passage_builder",
        "//onboard/planner/router:plot_util",
        "//onboard/planner/router:route_sections",
        "//onboard/planner/scene/proto:scene_understanding_cc_proto",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/speed:speed_finder",
        "//onboard/planner/speed:speed_finder_input",
        "//onboard/planner/speed:speed_finder_output",
        "//onboard/planner/util:planner_status_macros",
        "//onboard/planner/util:trajectory_plot_util",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:perception_cc_proto",
        "//onboard/vis/common:colormap",
    ],
)

cc_library(
    name = "est_planner_output",
    hdrs = ["est_planner_output.h"],
    deps = [
        ":trajectory_end_info",
        "//offboard/planner/ml/params_tuning/dopt_auto_tuning/proto:auto_tuning_cc_proto",
        "//onboard/planner/decision/proto:constraint_cc_proto",
        "//onboard/planner/initializer/proto:initializer_cc_proto",
        "//onboard/planner/object:partial_spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_object_trajectory",
        "//onboard/planner/object:spacetime_trajectory_manager",
        "//onboard/planner/speed/proto:speed_finder_cc_proto",
        "//onboard/proto:charts_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "driving_state",
    srcs = ["driving_state.cc"],
    hdrs = ["driving_state.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/maps:lane_path",
        "//onboard/planner/freespace/proto:freespace_planner_cc_proto",
        "//onboard/proto:planner_cc_proto",
    ],
)

cc_library(
    name = "teleop_state",
    srcs = ["teleop_state.cc"],
    hdrs = ["teleop_state.h"],
    deps = [
        ":planner_flags",
        "//onboard/proto:autonomy_state_cc_proto",
        "//onboard/proto:remote_assist_cc_proto",
        "//onboard/utils:proto_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "min_length_path_extension",
    srcs = ["min_length_path_extension.cc"],
    hdrs = ["min_length_path_extension.h"],
    deps = [
        ":discretized_path",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/planner/scheduler:scheduler_output",
        "//onboard/planner/util:path_util",
        "//onboard/proto:planner_cc_proto",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "planner_semantic_map_manager",
    srcs = ["planner_semantic_map_manager.cc"],
    hdrs = ["planner_semantic_map_manager.h"],
    deps = [
        ":planner_flags",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math/geometry:polygon2d",
        "//onboard/math/geometry:util",
        "@com_google_absl//absl/container:flat_hash_map",
    ],
)

cc_library(
    name = "trajectory_end_info",
    hdrs = ["trajectory_end_info.h"],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "trajectory_util_test",
    srcs = ["trajectory_util_test.cc"],
    deps = [
        ":trajectory_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "smooth_lane_test",
    srcs = ["smooth_lane_test.cc"],
    deps = [
        ":smooth_lane",
        "//onboard/global:test_main",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "speed_profile_test",
    srcs = ["speed_profile_test.cc"],
    deps = [
        ":speed_profile",
        "//onboard/global:test_main",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "discretized_path_test",
    srcs = ["discretized_path_test.cc"],
    deps = [
        ":discretized_path",
        "//onboard/global:test_main",
        "//onboard/math:frenet_frame",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "smooth_lane_path_test",
    srcs = ["smooth_lane_path_test.cc"],
    deps = [
        ":smooth_lane",
        ":smooth_lane_path",
        "//onboard/global:test_main",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/planner/router:route",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "composite_lane_path_test",
    srcs = ["composite_lane_path_test.cc"],
    tags = ["need_qcraft_map"],
    deps = [
        ":composite_lane_path",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "//onboard/maps:semantic_map_manager",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_state_test",
    srcs = ["planner_state_test.cc"],
    deps = [
        ":planner_state",
        ":planner_util",
        "//onboard/global:test_main",
        "//onboard/math:test_util",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "trajectory_validation_error_test",
    srcs = ["trajectory_validation_error_test.cc"],
    deps = [
        ":trajectory_validation_error",
        "//onboard/global:test_main",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "emergency_stop_test",
    srcs = ["emergency_stop_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":emergency_stop",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/global:test_main",
        "//onboard/planner/test_util:perception_object_builder",
        "//onboard/planner/test_util:util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "teleop_state_test",
    srcs = ["teleop_state_test.cc"],
    deps = [
        ":planner_flags",
        ":teleop_state",
        "//onboard/global:test_main",
        "//onboard/utils:proto_util",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "planner_semantic_map_manager_test",
    srcs = ["planner_semantic_map_manager_test.cc"],
    deps = [
        ":planner_flags",
        ":planner_semantic_map_manager",
        ":planner_util",
        "//onboard/global:test_main",
        "//onboard/maps:map_selector",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "min_length_path_extension_test",
    srcs = ["min_length_path_extension_test.cc"],
    deps = [
        ":min_length_path_extension",
        "//onboard/global:test_main",
        "//onboard/planner/util:path_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
