load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "elements_history",
    hdrs = ["elements_history.h"],
    deps = [
        "//onboard/async:async_util",
        "//onboard/utils:map_util",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/types:span",
    ],
)

cc_library(
    name = "release_util",
    srcs = [
        "release_util.cc",
    ],
    hdrs = [
        "release_util.h",
    ],
    data = [
        "//scripts:map_version",
    ],
    deps = [
        "//onboard/global:constants",
        "//onboard/logging/proto:lite_run_cc_proto",
        "//onboard/maps:map_selector",
        "//onboard/utils:file_util",
        "@com_github_nlohmann_json//:json",
    ],
)

cc_library(
    name = "periodic_runner",
    hdrs = ["periodic_runner.h"],
    deps = [
        "//onboard/global:car_common",
        "//onboard/lite:check",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "status_macros",
    hdrs = ["status_macros.h"],
    deps = [
        ":status_builder",
        "//onboard/base",
        "//onboard/lite:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_library(
    name = "process_util",
    srcs = ["process_util.cc"],
    hdrs = ["process_util.h"],
    deps = [
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "sys_util",
    srcs = ["sys_util.cc"],
    hdrs = ["sys_util.h"],
    deps = [
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "errors",
    hdrs = ["errors.h"],
    deps = [
        "//onboard/base",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "temp_file",
    srcs = ["temp_file.cc"],
    hdrs = ["temp_file.h"],
    deps = [
        ":errors",
        "//onboard/utils:status_macros",
        "@boost//:filesystem",
        "@boost//:system",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "map_util",
    hdrs = ["map_util.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/utils:source_location",
    ],
)

cc_library(
    name = "time_util",
    srcs = ["time_util.cc"],
    hdrs = ["time_util.h"],
    deps = [
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "string_util",
    hdrs = ["string_util.h"],
    deps = [
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "proto_util",
    srcs = ["proto_util.cc"],
    hdrs = ["proto_util.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "@com_google_absl//absl/strings",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "history_buffer",
    hdrs = ["history_buffer.h"],
    deps = [
        "//onboard/lite:check",
        "@boost//:circular_buffer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "objects_view",
    srcs = ["objects_view.cc"],
    hdrs = ["objects_view.h"],
    deps = [
        "//onboard/lite:check",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "labels_view",
    srcs = ["labels_view.cc"],
    hdrs = ["labels_view.h"],
    deps = [
        "//offboard/labeling/proto:label_frame_cc_proto",
        "//offboard/simulation/label:label_frame_filter",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:check",
        "//onboard/params:param_manager",
        "//onboard/utils:map_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "run_util",
    srcs = ["run_util.cc"],
    hdrs = ["run_util.h"],
    deps = [
        "//offboard/vfs",
        "//onboard/global:constants",
        "//onboard/lite:check",
        "@boost//:filesystem",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "file_util",
    srcs = ["file_util.cc"],
    hdrs = ["file_util.h"],
    deps = [
        ":filesystem",
        "//onboard/global:buffered_logger",
        "//onboard/lite:check",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "progress_bar",
    srcs = ["progress_bar.cc"],
    hdrs = ["progress_bar.h"],
    deps = [
        "//onboard/lite:check",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "interval_tree",
    hdrs = ["interval_tree.h"],
)

cc_library(
    name = "stacktrace",
    srcs = ["stacktrace.cc"],
    hdrs = ["stacktrace.h"],
    deps = [
        "@com_google_absl//absl/debugging:stacktrace",
        "@com_google_absl//absl/debugging:symbolize",
    ],
)

cc_library(
    name = "debug_utils",
    hdrs = [
        "fps_counter.h",
        "log_to_file.h",
    ],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "keyboard_scanner",
    srcs = ["keyboard_scanner.cc"],
    hdrs = ["keyboard_scanner.h"],
    deps = [
        ":thread_util",
    ],
)

cc_library(
    name = "lite_msg_stats",
    srcs = ["lite_msg_stats.cc"],
    hdrs = ["lite_msg_stats.h"],
    deps = [
        ":time_util",
        "@boost//:circular_buffer",
    ],
)

cc_library(
    name = "terminal_color",
    hdrs = ["terminal_color.h"],
)

cc_library(
    name = "cuda_util",
    hdrs = ["cuda_util.h"],
    deps = [
        "@cuda",
    ],
)

cc_library(
    name = "source_location",
    hdrs = ["source_location.h"],
    deps = [
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "status_builder",
    srcs = ["status_builder.cc"],
    hdrs = ["status_builder.h"],
    deps = [
        ":source_location",
        "//onboard/utils/proto:codes_cc_proto",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "test_util",
    testonly = True,
    hdrs = ["test_util.h"],
    deps = [
        ":proto_util",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "id_generator",
    srcs = [
        "id_generator.cc",
    ],
    hdrs = [
        "id_generator.h",
    ],
    deps = [
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "archive_util",
    srcs = ["archive_util.cc"],
    hdrs = ["archive_util.h"],
    deps = [
        "//onboard/utils:filesystem",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/types:span",
        "@libarchive",
    ],
)

cc_library(
    name = "file_util_mmap",
    srcs = ["file_util_mmap.cc"],
    hdrs = ["file_util_mmap.h"],
    deps = [
        "//onboard/global:singleton",
        "//onboard/lite:check",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "filesystem",
    hdrs = ["filesystem.h"],
)

cc_library(
    name = "network_card_monitor",
    srcs = ["network_card_monitor.cc"],
    hdrs = ["network_card_monitor.h"],
    deps = [
        "//onboard/base:integral_types",
        "@boost//:algorithm",
        "@boost//:process",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "http_client",
    srcs = ["http_client.cc"],
    hdrs = ["http_client.h"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_github_jpbarrette_curlpp//:curlpp",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/status",
    ],
)

cc_library(
    name = "thread_util",
    srcs = ["thread_util.cc"],
    hdrs = ["thread_util.h"],
)

###############################################################################
# Binary.

cc_binary(
    name = "periodic_runner_main",
    srcs = ["periodic_runner_main.cc"],
    deps = [
        ":periodic_runner",
        "//onboard/global:init_qcraft",
    ],
)

cc_binary(
    name = "id_generator_main",
    srcs = ["id_generator_main.cc"],
    deps = [
        ":id_generator",
        "//onboard/global:init_qcraft",
    ],
)

cc_binary(
    name = "circular_buffer_bm",
    srcs = ["circular_buffer_bm.cc"],
    deps = [
        "@boost//:circular_buffer",
        "@com_github_google_benchmark//:benchmark_main",
    ],
)

cc_binary(
    name = "flat_hash_map_bm",
    srcs = ["flat_hash_map_bm.cc"],
    deps = [
        "@com_github_google_benchmark//:benchmark_main",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/container:node_hash_map",
    ],
)

cc_binary(
    name = "file_util_mmap_bm",
    srcs = ["file_util_mmap_bm.cc"],
    deps = [
        ":file_util",
        ":file_util_mmap",
        "//onboard/lite:logging",
        "@com_github_google_benchmark//:benchmark_main",
    ],
)

###############################################################################
# Tests.
cc_test(
    name = "elements_history_test",
    srcs = ["elements_history_test.cc"],
    deps = [
        ":elements_history",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "file_util_test",
    size = "small",
    srcs = ["file_util_test.cc"],
    deps = [
        ":file_util",
        "//common/proto:lane_point_cc_proto",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "file_util_mmap_test",
    size = "small",
    srcs = ["file_util_mmap_test.cc"],
    deps = [
        ":file_util",
        ":file_util_mmap",
        "//onboard/lite:check",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "release_util_test",
    srcs = ["release_util_test.cc"],
    data = glob([
        "testdata/release_util/**",
    ]),
    deps = [
        ":release_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "time_util_test",
    srcs = ["time_util_test.cc"],
    deps = [
        ":time_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "objects_view_test",
    srcs = ["objects_view_test.cc"],
    deps = [
        ":objects_view",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lite_msg_stats_test",
    srcs = ["lite_msg_stats_test.cc"],
    deps = [
        ":lite_msg_stats",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "history_buffer_test",
    srcs = ["history_buffer_test.cc"],
    deps = [
        ":history_buffer",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "progress_bar_test",
    srcs = ["progress_bar_test.cc"],
    deps = [
        ":progress_bar",
        "//onboard/async:parallel_for",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "status_macros_test",
    size = "small",
    srcs = ["status_macros_test.cc"],
    deps = [
        ":status_macros",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "sys_util_test",
    srcs = ["sys_util_test.cc"],
    deps = [
        ":sys_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "id_generator_test",
    srcs = [
        "id_generator_test.cc",
    ],
    deps = [
        ":id_generator",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "archive_util_test",
    srcs = ["archive_util_test.cc"],
    data = glob([
        "testdata/archive_util/**",
    ]),
    tags = ["manual"],
    deps = [
        ":archive_util",
        "@boost//:filesystem",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "source_location_test",
    srcs = ["source_location_test.cc"],
    deps = [
        ":source_location",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "map_util_test",
    srcs = ["map_util_test.cc"],
    deps = [
        ":map_util",
        "//onboard/global:test_main",
        "@com_google_googletest//:gtest",
    ],
)

cc_binary(
    name = "archive_util_bm",
    srcs = ["archive_util_bm.cc"],
    data = glob([
        "testdata/archive_util/**",
    ]),
    tags = ["bm"],
    deps = [
        ":archive_util",
        "@boost//:filesystem",
        "@com_github_google_benchmark//:benchmark_main",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
