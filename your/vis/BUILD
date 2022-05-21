load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "vantage_forwarding_module",
    srcs = [
        "vantage_forwarding_module.cc",
    ],
    hdrs = ["vantage_forwarding_module.h"],
    deps = [
        "//offboard/vis/ark/ark_server:ark_client_man",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "//onboard/lidar:spin_reader",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:spin_util",
        "//onboard/lite:check",
        "//onboard/lite:lite_module",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/lite:module_config_helper",
        "//onboard/logging:stf_reader",
        "//onboard/maps:maps_helper",
        "//onboard/perception/registration:icp",
        "//onboard/proto:lidar_cc_proto",
        "//onboard/proto:lite_msg_cc_proto",
        "//onboard/proto:port_cc_proto",
        "//onboard/proto:positioning_cc_proto",
        "//onboard/remote_assist:udp_update_client",
        "//onboard/utils:file_util",
        "//onboard/utils:periodic_runner",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "http_request",
    srcs = ["http_request.cc"],
    hdrs = ["http_request.h"],
)

cc_library(
    name = "json_lookahead_parser",
    srcs = ["json_lookahead_parser.cc"],
    hdrs = ["json_lookahead_parser.h"],
    deps = [
        "@rapidjson",
    ],
)

cc_library(
    name = "qview_route_info",
    srcs = ["qview_route_info.cc"],
    hdrs = ["qview_route_info.h"],
    deps = [
        ":json_lookahead_parser",
        "//onboard/global:trace",
        "@rapidjson",
    ],
)

cc_library(
    name = "qview_data_serializer",
    srcs = ["qview_data_serializer.cc"],
    hdrs = ["qview_data_serializer.h"],
    deps = [
        ":qview_route_info",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/proto:hmi_content_cc_proto",
        "//onboard/proto:port_cc_proto",
        "//onboard/proto:qview_cc_proto",
        "@rapidjson",
    ],
)

cc_library(
    name = "websockets_server",
    srcs = ["websockets_server.cc"],
    hdrs = ["websockets_server.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/utils:thread_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/synchronization",
        "@websocketpp",
    ],
)

cc_library(
    name = "qview_publisher_module",
    srcs = ["qview_publisher_module.cc"],
    hdrs = ["qview_publisher_module.h"],
    deps = [
        ":qview_data_serializer",
        ":qview_route_info",
        ":websockets_server",
        "//onboard/async:async_util",
        "//onboard/camera/utils:image_util",
        "//onboard/crypto:base64",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:lite_module",
        "//onboard/maps:semantic_map_manager",
        "//onboard/math:coordinate_converter",
        "//onboard/perception:obstacle",
        "//onboard/planner:composite_lane_path",
        "//onboard/proto:port_cc_proto",
        "//onboard/proto:q_run_event_cc_proto",
        "//onboard/proto:qview_cc_proto",
        "//onboard/utils:history_buffer",
        "//onboard/utils:thread_util",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/container:flat_hash_map",
        "@opencv",
    ],
)

cc_library(
    name = "qshow_publisher_module",
    srcs = ["qshow_publisher_module.cc"],
    hdrs = ["qshow_publisher_module.h"],
    deps = [
        ":http_request",
        "//onboard/async:async_util",
        "//onboard/camera/utils:camera_image",
        "//onboard/camera/utils:camera_util",
        "//onboard/crypto:base64",
        "//onboard/global:trace",
        "//onboard/lite:lite_module",
        "//onboard/lite:lite_shm_message",
        "//onboard/lite:logging",
        "//onboard/nets:fiery_eye_net_classifier",
        "//onboard/nets:panonet",
        "//onboard/nets/proto:net_param_cc_proto",
        "//onboard/perception:projection_util",
        "//onboard/proto:port_cc_proto",
        "//onboard/proto:qview_cc_proto",
        "//onboard/vis/qshow_process:fen_tracker",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
        "@opencv",
    ],
)

cpplint()
