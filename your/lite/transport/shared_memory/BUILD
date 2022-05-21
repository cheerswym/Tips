load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "lite_shared_memory_manager",
    srcs = ["lite_shared_memory_block.cc"],
    hdrs = [
        "lite_shared_memory_block.h",
        "lite_shared_memory_manager.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:shm_message_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "lite_inner_process_shared_memory_manager",
    srcs = ["lite_inner_process_shared_memory_manager.cc"],
    hdrs = ["lite_inner_process_shared_memory_manager.h"],
    deps = [
        ":lite_shared_memory_manager",
        "//onboard/base",
        "//onboard/global:counter",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:shm_message_cc_proto",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/strings",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "lite_shared_memory_manager_factory",
    srcs = ["lite_shared_memory_manager_factory.cc"],
    hdrs = ["lite_shared_memory_manager_factory.h"],
    deps = [
        ":lite_inner_process_shared_memory_manager",
        ":lite_shared_memory_manager",
        ":lite_shm_shared_memory_manager",
        "//onboard/lite:lite2_flags",
    ],
)

cc_library(
    name = "lite_shm_shared_memory_manager",
    srcs = ["lite_shm_shared_memory_manager.cc"],
    hdrs = ["lite_shm_shared_memory_manager.h"],
    deps = [
        ":lite_shared_memory_manager",
        "//onboard/base",
        "//onboard/global:counter",
        "//onboard/lite:lite_message_util",
        "//onboard/lite/proto:lite_common_cc_proto",
        "//onboard/lite/proto:shm_message_cc_proto",
        "//onboard/lite/transport/shm:shm_manager",
        "//onboard/utils:map_util",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/synchronization",
        "@com_google_absl//absl/time",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.
cc_test(
    name = "lite_shm_shared_memory_manager_test",
    srcs = ["lite_shm_shared_memory_manager_test.cc"],
    deps = [
        ":lite_shm_shared_memory_manager",
        "//onboard/lite/transport/shm:lite_shm_header",
        "//onboard/lite/transport/shm:shm_manager",
        "//onboard/lite/transport/shm:shm_transmitter",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lite_shared_memory_manager_factory_test",
    srcs = ["lite_shared_memory_manager_factory_test.cc"],
    deps = [
        ":lite_shared_memory_manager_factory",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lite_shared_memory_block_test",
    srcs = ["lite_shared_memory_block_test.cc"],
    deps = [
        ":lite_inner_process_shared_memory_manager",
        ":lite_shared_memory_manager",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lite_inner_process_shared_memory_manager_test",
    srcs = ["lite_inner_process_shared_memory_manager_test.cc"],
    deps = [
        ":lite_inner_process_shared_memory_manager",
        "//onboard/lite/transport/shm:shm_manager",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
