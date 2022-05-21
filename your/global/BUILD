load("@bazel_skylib//lib:selects.bzl", "selects")
load("//bazel:common.bzl", "if_aarch64", "if_x86_64")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "ftrace_data",
    data = glob([
        "data/*.ftrace",
    ]),
)

###############################################################################
# Libraries.

cc_library(
    name = "singleton",
    hdrs = ["singleton.h"],
)

cc_library(
    name = "logging",
    hdrs = ["logging.h"],
    deps = ["//onboard/base"],
)

cc_library(
    name = "buffered_logger",
    srcs = ["buffered_logger.cc"],
    hdrs = ["buffered_logger.h"],
    deps = [
        ":car_common",
        ":singleton",
        ":spin_lock",
        "//onboard/proto:buffered_log_cc_proto",
        "//onboard/utils:stacktrace",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

cc_library(
    name = "constants",
    srcs = ["constants.cc"],
    hdrs = ["constants.h"],
)

cc_library(
    name = "port_constants",
    srcs = ["port_constants.cc"],
    hdrs = ["port_constants.h"],
    deps = [
        "//onboard/proto:port_cc_proto",
    ],
)

cc_library(
    name = "system_cpu",
    srcs = ["system_cpu.cc"],
    hdrs = ["system_cpu.h"],
)

cc_library(
    name = "system_mem",
    srcs = ["system_mem.cc"],
    hdrs = ["system_mem.h"],
)

cc_library(
    name = "system_io",
    srcs = ["system_io.cc"],
    hdrs = ["system_io.h"],
)

cc_library(
    name = "system_misc",
    srcs = if_x86_64(
        [
            "system_misc.cc",
        ],
        [
            "system_misc_null.cc",
        ],
    ),
    hdrs = ["system_misc.h"],
    deps = [
        "//onboard/proto:system_info_cc_proto",
        "//onboard/utils:file_util",
        "@boost//:filesystem",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "system_info",
    srcs = ["system_info.cc"] + selects.with_or({
        ("//bazel:cpu_only", "//bazel:linux_aarch64_cross_compile"): [
            "system_gpu_null.cc",
        ],
        "//bazel:linux_aarch64": ["system_gpu_xavier.cc"],
        "//bazel:linux_x86_64": ["system_gpu.cc"],
    }),
    hdrs = ["system_info.h"],
    deps = [
        ":singleton",
        ":system_cpu",
        ":system_mem",
        ":system_io",
        ":system_misc",
        "@com_google_absl//absl/strings:str_format",
        "//onboard/node:node_util",
        "//onboard/proto:system_info_cc_proto",
        "//onboard/proto:system_status_info_cc_proto",
        "//onboard/utils:file_util",
        "//onboard/utils:map_util",
        "@boost//:filesystem",
        "@com_google_absl//absl/strings",
    ] + selects.with_or({
        ("//bazel:cpu_only", "//bazel:linux_aarch64"): [
            "@cuda",
        ],
        "//bazel:linux_aarch64_cross_compile": [
            "@cuda",
        ],
        "//conditions:default": [
            "@cuda",
            "@nvml",
        ],
    }),
)

cc_library(
    name = "clock",
    srcs = ["clock.cc"],
    hdrs = ["clock.h"],
    deps = [
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "timer",
    srcs = ["timer.cc"],
    hdrs = ["timer.h"],
    deps = [
        "//onboard/utils:thread_util",
        "@boost//:asio",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
    ],
)

cc_library(
    name = "spin_lock",
    hdrs = ["spin_lock.h"],
    deps = ["//onboard/base"],
)

cc_library(
    name = "trace_util",
    srcs = ["trace_util.cc"],
    hdrs = ["trace_util.h"],
    deps = [
        "//onboard/proto:trace_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@zlib",
    ],
)

cc_library(
    name = "ftrace",
    srcs = ["ftrace.cc"],
    hdrs = ["ftrace.h"],
    deps = [
        ":singleton",
        "//onboard/base",
        "//onboard/global:trace_util",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/utils:map_util",
        "//onboard/utils:thread_util",
        "@boost//:interprocess",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

cc_library(
    name = "trace",
    srcs = ["trace.cc"],
    hdrs = ["trace.h"],
    deps = [
        ":counter",
        ":ftrace",
        ":spin_lock",
        "//onboard/base",
        "//onboard/lite:check",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/node:node_util",
        "//onboard/proto:trace_cc_proto",
        "//onboard/utils:time_util",
        "@boost//:circular_buffer",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

cc_library(
    name = "counter",
    srcs = ["counter.cc"],
    hdrs = ["counter.h"],
    deps = [
        ":singleton",
        ":spin_lock",
        "//onboard/base",
        "//onboard/math:util",
        "//onboard/proto:counter_cc_proto",
        "//onboard/utils:map_util",
        "@boost//:circular_buffer",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@folly",
    ],
)

cc_library(
    name = "registry",
    hdrs = ["registry.h"],
    deps = ["//onboard/lite:check"],
)

cc_library(
    name = "init_qcraft",
    srcs = ["init_qcraft.cc"],
    hdrs = ["init_qcraft.h"],
    deps = [
        "//onboard/maps:map_selector",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/debugging:failure_signal_handler",
        "@com_google_absl//absl/debugging:symbolize",
        "@folly",
    ],
)

cc_library(
    name = "run_context",
    srcs = ["run_context.cc"],
    hdrs = ["run_context.h"],
    deps = [
        "//onboard/base:base_flags",
        "//onboard/lite:lite2_flags",
        "//onboard/lite:sensor_scenario_config_helper",
        "//onboard/params:param_manager",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "car_common",
    srcs = ["car_common.cc"],
    hdrs = ["car_common.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "test_main",
    testonly = True,
    srcs = ["test_main.cc"],
    deps = [
        ":init_qcraft",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "system_status",
    srcs = ["system_status.cc"],
    hdrs = ["system_status.h"],
    deps = [
        "//onboard/camera/v3:camera_util_v4l2",
        "//onboard/global:system_info",
        "//onboard/proto:system_status_info_cc_proto",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "@boost//:filesystem",
        "@boost//:process",
        "@boost//:system",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "protobuf_flags",
    srcs = ["protobuf_flags.cc"],
    hdrs = ["protobuf_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "protobuf_arena_manager",
    hdrs = ["protobuf_arena_manager.h"],
    deps = [
        ":counter",
        ":protobuf_flags",
        "@com_github_google_glog//:glog",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Tests.

cc_test(
    name = "registry_test",
    srcs = ["registry_test.cc"],
    deps = [
        ":registry",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "logging_test",
    srcs = ["logging_test.cc"],
    deps = [
        ":logging",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "timer_test",
    srcs = ["timer_test.cc"],
    deps = [
        ":timer",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "clock_test",
    srcs = ["clock_test.cc"],
    deps = [
        ":clock",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "counter_test",
    srcs = ["counter_test.cc"],
    deps = [
        ":counter",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "ftrace_test",
    srcs = ["ftrace_test.cc"],
    deps = [
        ":ftrace",
        ":init_qcraft",
        ":trace_util",
        "//onboard/global:test_main",
        "//onboard/utils:temp_file",
        "@boost//:filesystem",
        "@boost//:process",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "run_context_test",
    srcs = ["run_context_test.cc"],
    deps = [
        ":run_context",
        "//onboard/global:test_main",
        "//onboard/params/utils:vehicle_params_flags",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "system_info_test",
    srcs = ["system_info_test.cc"],
    deps = [
        ":system_info",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trace_util_test",
    srcs = ["trace_util_test.cc"],
    data = [
        ":ftrace_data",
    ],
    deps = [
        ":init_qcraft",
        ":trace_util",
        "//onboard/global:test_main",
        "//onboard/utils:temp_file",
        "@boost//:filesystem",
        "@boost//:process",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/strings:str_format",
        "@com_google_googletest//:gtest",
    ],
)

# omc: bazel run -c opt  onboard/global/system_status_test
# xavier: bazel run -c opt onboard/global/system_status_test
cc_test(
    name = "system_status_test",
    srcs = if_aarch64(
        [
            "system_status_xavier_test.cc",
        ],
        [
            "system_status_test.cc",
        ],
    ),
    deps = [
        ":system_status",
        "//onboard/proto:vehicle_cc_proto",
        "//onboard/utils:file_util",
        "@boost//:filesystem",
        "@boost//:process",
        "@boost//:system",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "protobuf_arena_manager_test",
    srcs = ["protobuf_arena_manager_test.cc"],
    deps = [
        ":protobuf_arena_manager",
        "//experimental/users/youliang/proto_buffer/proto:test_cc_proto",
        "//onboard/proto:system_info_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
