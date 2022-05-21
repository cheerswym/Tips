load("//bazel:common.bzl", "if_x86_64")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")
load("//bazel/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "lite_timer",
    srcs = ["lite_timer.cc"],
    hdrs = ["lite_timer.h"],
    deps = [
        "//onboard/async:thread_pool",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/proto:execution_issue_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
        "@folly",
    ],
)

cc_library(
    name = "lite_shm_message",
    # srcs = ["lite_shm_block.cc"],
    hdrs = [
        "lite_shm_message.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/global:trace",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:shm_message_cc_proto",
        "//onboard/lite/transport/shared_memory:lite_shared_memory_manager_factory",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "lite_message_util",
    srcs = ["lite_message_util.cc"],
    hdrs = [
        "lite_message_util.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "aggregate_lite_message_util",
    srcs = ["aggregate_lite_message_util.cc"],
    hdrs = [
        "aggregate_lite_message_util.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "lite_callbacks",
    hdrs = [
        "lite_callbacks.h",
        "lite_message_util.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/utils:errors",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "transport",
    hdrs = [
        "transport.h",
    ],
    deps = [
        ":lite_callbacks",
        "//onboard/lite/modifier:message_modifier_man",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/transport/message:message_hub",
        "//onboard/proto:execution_issue_cc_proto",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "lite_transport",
    srcs = ["lite_transport.cc"],
    hdrs = [
        "lite_transport.h",
    ],
    deps = [
        ":lite_callbacks",
        ":transport",
        "//onboard/async:parallel_for",
        "//onboard/global:car_common",
        "//onboard/global:clock",
        "//onboard/global:counter",
        "//onboard/global:trace",
        "//onboard/lite:logging",
        "//onboard/lite/modifier:message_modifier_man",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/transport/message:message_hub",
        "//onboard/lite/transport/shm:lite_shm_header",
        "//onboard/lite/transport/shm:shm_debug_util",
        "//onboard/lite/transport/shm:shm_manager",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "check",
    hdrs = [
        "check.h",
        "check_helper.h",
    ],
    deps = [
        ":lite2_flags",
        "//onboard/global:buffered_logger",
        "//onboard/global:logging",
        "//onboard/proto:buffered_log_cc_proto",
    ],
)

cc_library(
    name = "logging",
    hdrs = ["logging.h"],
    deps = [
        ":check",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "qissue_trans",
    srcs = ["qissue_trans.cc"],
    hdrs = ["qissue_trans.h"],
    deps = [
        ":lite_client",
        ":logging",
        "//offboard/simulation/lite:sim_lite_client",
        "//onboard/autonomy_state/handlers:q_issue_util",
    ],
)

cc_library(
    name = "lite_module",
    srcs = ["lite_module.cc"],
    hdrs = [
        "lite_module.h",
    ],
    deps = [
        ":lite2_flags",
        ":lite_client",
        ":lite_client_base",
        ":module_config_helper",
        ":sensor_scenario_config_helper",
        ":transport",
        "//onboard/autonomy_state:autonomy_state_util",
        "//onboard/autonomy_state/handlers:q_issue_util",
        "//onboard/base",
        "//onboard/base:base_flags",
        "//onboard/camera/utils:camera_image",
        "//onboard/global:car_common",
        "//onboard/global:registry",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/lidar:spin_structs",
        "//onboard/lite:logging",
        "//onboard/lite:qissue_trans",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/scheduler/common:cpu_affinity",
        "//onboard/lite/transport/shared_memory:lite_shared_memory_manager_factory",
        "//onboard/lite/transport/shm:shm_manager",
        "//onboard/params:param_manager",
        "//onboard/proto:sensor_scenario_config_cc_proto",
        "//onboard/utils:errors",
        "//onboard/utils:thread_util",
        "@boost//:bimap",
        "@boost//:circular_buffer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "lite_client_base",
    srcs = ["lite_client_base.cc"],
    hdrs = ["lite_client_base.h"],
    deps = [
        ":lite_callbacks",
        ":lite_message_util",
        ":lite_shm_message",
        ":lite_timer",
        "//onboard/async:thread_pool",
        "//onboard/global:clock",
        "//onboard/lite:check",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/lite/proto:shm_message_cc_proto",
        "//onboard/lite/transport/message:message_hub",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
        "@folly",
    ],
)

cc_library(
    name = "lite_client",
    srcs = ["lite_client.cc"],
    hdrs = [
        "lite_client.h",
    ],
    deps = [
        ":lite_callbacks",
        ":lite_client_base",
        ":lite_transport",
        ":transport",
        "//onboard/global:clock",
        "//onboard/lite/proto:lite_common_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "schedule_test_module",
    hdrs = ["schedule_test_module.h"],
    deps = [
        ":lite_module",
        "//onboard/global:clock",
    ],
)

cc_library(
    name = "module_config_helper",
    srcs = [
        "module_config_helper.cc",
    ],
    hdrs = ["module_config_helper.h"],
    data = [
        "//onboard/lite/launch_config:launch_config_files",
        "//onboard/lite/launch_config:module_config_files",
    ],
    deps = [
        "//onboard/base",
        "//onboard/global:car_common",
        "//onboard/lite:check",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "@boost//:filesystem",
        "@boost//:range",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "sensor_scenario_config_helper",
    srcs = [
        "sensor_scenario_config_helper.cc",
    ],
    hdrs = ["sensor_scenario_config_helper.h"],
    data = [
        "//onboard/lite/scenario_config:sensor_scenario_config_files",
    ],
    deps = [
        "//onboard/base",
        "//onboard/global:car_common",
        "//onboard/lite:check",
        "//onboard/proto:sensor_scenario_config_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "@boost//:filesystem",
        "@boost//:range",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "lite2_flags",
    srcs = [
        "lite2_flags.cc",
    ],
    hdrs = ["lite2_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "onboard_modules",
    hdrs = [
        "onboard_modules.h",
    ],
    deps = select({
        "//bazel:linux_aarch64": [
            "//onboard/camera:image_publisher_module",
            "//onboard/camera:image_sender_module",
            "//onboard/can:can_bus_module",
            "//onboard/localization:localization_module",
            "//onboard/logging:logging_module",
            "//onboard/node:node_module",
            "//onboard/node:node_state_module",
            "//onboard/positioning:positioning_module",
            "//onboard/positioning/drivers:gnss_imu_driver_module",
            "//onboard/radar:radar_process_module",
            "//onboard/radar:radar_publisher_module",
            "//onboard/rsim/playback:rsim_playback_module",
            "//onboard/rsim/state_monitor:rsim_state_monitor_module",
            "//onboard/rsim/vehicle:rsim_vehicle_module",
            "//onboard/vis:qview_publisher_module",
            "//onboard/vis:vantage_forwarding_module",
        ],
        "//bazel:linux_aarch64_cross_compile": [
            "//onboard/autonomy_state:autonomy_state_module",
            "//onboard/can:can_bus_module",
            "//onboard/control:vehicle_control_module",
            "//onboard/control/calibration:control_calibration_module",
            "//onboard/emergency_brake:emergency_brake_module",
            "//onboard/hmi:autonomy_hmi_module",
            "//onboard/localization:localization_module",
            "//onboard/logging:logging_module",
            "//onboard/lidar:spin_publisher_module",
            "//onboard/node:node_state_module",
            "//onboard/node:node_module",
            "//onboard/occ/host:occ_module",
            "//onboard/perception:perception_module",
            "//onboard/perception:tracker_module",
            # "//onboard/perception:vision_module",
            # "//onboard/planner:planner_module",
            "//onboard/positioning/drivers:gnss_imu_driver_module",
            "//onboard/positioning:positioning_module",
            "//onboard/radar:radar_process_module",
            "//onboard/radar:radar_publisher_module",
            # "//onboard/rsim/playback:rsim_playback_module",
            # "//onboard/rsim/vehicle:rsim_vehicle_module",
            # "//onboard/vis:vantage_forwarding_module",
        ],
        "//bazel:linux_x86_64": [
            "//onboard/camera:image_publisher_module",
            "//onboard/can:can_bus_module",
            "//onboard/autonomy_state:autonomy_state_module",
            "//onboard/hmi:autonomy_hmi_module",
            "//onboard/control:vehicle_control_module",
            "//onboard/control/calibration:control_calibration_module",
            "//onboard/lidar:spin_publisher_module",
            "//onboard/lite:schedule_test_module",
            "//onboard/perception:perception_module",
            "//onboard/perception:vision_module",
            "//onboard/perception:tracker_module",
            "//onboard/planner:planner_module",
            "//onboard/positioning:positioning_module",
            "//onboard/positioning/drivers:gnss_imu_driver_module",
            "//onboard/radar:radar_publisher_module",
            "//onboard/radar:radar_process_module",
            "//onboard/remote_assist:remote_assist_module",
            "//onboard/sensor_live:sensor_live_module",
            "//onboard/simulation:online_simulation_module",
            "//onboard/simulation:test_module_state_module",
            "//onboard/v2x:v2x_module",
            "//onboard/vis:qshow_publisher_module",
            "//onboard/occ/host:occ_module",
            "//onboard/emergency_brake:emergency_brake_module",
            "//onboard/camera:image_sender_module",
            "//onboard/node:node_state_module",
            "//onboard/node:node_module",
            "//onboard/vis:qview_publisher_module",
            "//onboard/localization:localization_module",
            "//onboard/rsim/playback:rsim_playback_module",
            "//onboard/rsim/vehicle:rsim_vehicle_module",
            "//onboard/rsim/state_monitor:rsim_state_monitor_module",
            "//onboard/vis:vantage_forwarding_module",
            "//onboard/logging:logging_module",
        ],
        "//conditions:default": [
        ],
    }) + [
        "//onboard/camera:image_forwarding_module",
        "//onboard/planner/router:route_recorder_module",
        "//onboard/planner/router:routing_module",
    ],
)

cc_library(
    name = "offboard_modules",
    hdrs = ["offboard_modules.h"],
    deps = if_x86_64([
        "//offboard/hil_tests/pacmod:joystick_control_module",
        "//offboard/playback:interactive_playback_module",
        "//offboard/tools/analysis:data_analysis_module",
    ]),
)

cc_library(
    name = "mock_modules",
    hdrs = ["mock_modules.h"],
    deps = [
        "//onboard/logging:logging_module",
        "//onboard/mock:lite_performance_test_module",
        "//onboard/mock:mock_gnss_imu_driver_module",
        "//onboard/mock:mock_image_publish_module",
        "//onboard/mock:mock_lite_pressure_test_module",
        "//onboard/mock:mock_logging_module",
        "//onboard/mock:mock_pubsub_module",
        "//onboard/mock:mock_self_destruct_module",
        "//onboard/mock:mock_spin_publisher_module",
    ],
)
###############################################################################
# Binaries.

cc_binary(
    name = "launch_autonomy_main",
    srcs = [
        "launch_autonomy_main.cc",
    ],
    data = [
        "//offboard/calibration/data:lidar_extrinsics_file",
        "//onboard/lidar/data:lidar_common_params_data",
        "//onboard/lite/launch_config:launch_config_files",
        "//onboard/lite/launch_config:module_config_files",
    ],
    deps = [
        ":logging",
        "//onboard/global:buffered_logger",
        "//onboard/global:init_qcraft",
        "//onboard/global:logging",
        "//onboard/lite/runner:autonomy_runner",
        "//onboard/lite/runner:module_runner",
        "@com_google_absl//absl/debugging:failure_signal_handler",
    ],
)

cc_binary(
    name = "mini_autonomy_main",
    srcs = [
        "mini_autonomy_main.cc",
    ],
    deps = [
        ":logging",
        "//onboard/global:buffered_logger",
        "//onboard/global:init_qcraft",
        "//onboard/global:logging",
        "//onboard/lite/runner:autonomy_runner",
        "@com_google_absl//absl/debugging:failure_signal_handler",
    ],
)

cc_binary(
    name = "mock_autonomy_main",
    srcs = [
        "mock_autonomy_main.cc",
    ],
    deps = [
        ":logging",
        "//onboard/global:buffered_logger",
        "//onboard/global:init_qcraft",
        "//onboard/global:logging",
        "//onboard/lite/runner:autonomy_runner",
        "//onboard/lite/runner:mock_runner",
        "@com_google_absl//absl/debugging:failure_signal_handler",
    ],
)

cc_binary(
    name = "launch_car_main",
    srcs = [
        "launch_car_main.cc",
    ],
    data = [
        "//offboard/calibration/data:lidar_extrinsics_file",
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":lite_callbacks",
        ":lite_client",
        ":lite_client_base",
        ":lite_message_util",
        ":lite_shm_message",
        ":module_config_helper",
        ":offboard_modules",
        ":onboard_modules",
        "//onboard/async:parallel_for",
        "//onboard/base:base_flags",
        "//onboard/base:signal_handler",
        "//onboard/conf:run_conf_man",
        "//onboard/eval/collectors:qevent_log_collector",
        "//onboard/global:car_common",
        "//onboard/global:init_qcraft",
        "//onboard/lite/sidecar:ondemand_sidecar",
        "//onboard/lite/sidecar:periodic_sidecar",
        "//onboard/lite/transport/message:inner_message_hub",
        "//onboard/maps:map_selector",
        "//onboard/math:coordinate_converter",
        "//onboard/params:param_manager",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/debugging:stacktrace",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@folly",
    ],
)

install(
    name = "install",
    targets = [
        ":launch_car_main",
    ],
)

###############################################################################
# Tests.
# Client test
cc_test(
    name = "lite_client_test",
    srcs = [
        "lite_client_test.cc",
    ],
    deps = [
        ":lite_client",
        ":lite_shm_message",
        ":transport",
        "//onboard/lite:logging",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/testdata:lite_msg_test_cc_proto",
        "//onboard/lite/transport/message:inner_message_hub",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "transport_test",
    srcs = [
        "transport_test.cc",
    ],
    deps = [
        ":lite_timer",
        ":lite_transport",
        ":transport",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/transport/message:inner_message_hub",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lite_message_util_test",
    srcs = [
        "lite_message_util_test.cc",
    ],
    deps = [
        ":lite_message_util",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "aggregate_lite_message_util_test",
    srcs = [
        "aggregate_lite_message_util_test.cc",
    ],
    deps = [
        ":aggregate_lite_message_util",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "lite_client_base_test",
    srcs = [
        "lite_client_base_test.cc",
    ],
    deps = [
        ":lite_client_base",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "lite_timer_test",
    srcs = [
        "lite_timer_test.cc",
    ],
    deps = [
        ":lite_timer",
        "//onboard/proto:execution_issue_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@folly",
    ],
)

cc_test(
    name = "lite_shm_message_test",
    srcs = [
        "lite_shm_message_test.cc",
    ],
    deps = [
        ":lite_shm_message",
        "//onboard/lite/proto:lite_common_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "module_config_test",
    srcs = [
        "module_config_test.cc",
    ],
    data = [
        "//onboard/lite/launch_config:launch_config_files",
        "//onboard/lite/launch_config:module_config_files",
    ],
    deps = [
        ":lite_client",
        ":lite_client_base",
        ":lite_message_util",
        ":lite_module",
        ":module_config_helper",
        ":offboard_modules",
        ":onboard_modules",
        ":transport",
        "//offboard/hil_tests/pacmod:joystick_control_module",
        "//offboard/virtual_world:vehicle_engine_module",
        "//offboard/virtual_world:virtual_world_module",
        "//onboard/global:registry",
        "//onboard/lite/proto:module_config_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "sensor_scenario_config_test",
    srcs = [
        "sensor_scenario_config_test.cc",
    ],
    deps = [
        ":sensor_scenario_config_helper",
        "//onboard/global:registry",
        "//onboard/proto:sensor_scenario_config_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "qcheck_test",
    srcs = ["qcheck_test.cc"],
    deps = [
        "//onboard/lite:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "qissue_trans_test",
    srcs = ["qissue_trans_test.cc"],
    deps = [
        ":lite_client",
        ":lite_module",
        ":lite_transport",
        ":qissue_trans",
        ":transport",
        "//onboard/global:clock",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/transport/message:inner_message_hub",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
