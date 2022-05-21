load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shm_debug_util",
    srcs = [
        "shm_debug_util.cc",
    ],
    hdrs = [
        "shm_debug_util.h",
    ],
    deps = [
        "//onboard/base",
        "//onboard/global:singleton",
        "//onboard/lite:lite_message_util",
        "//onboard/lite:module_config_helper",
        "//onboard/utils:map_util",
    ],
)

cc_library(
    name = "shm_manager",
    srcs = ["shm_manager.cc"],
    hdrs = ["shm_manager.h"],
    linkopts = ["-lrt"],
    deps = [
        ":lite_shm_header",
        ":shm_debug_util",
        ":shm_flags",
        "//onboard/base",
        "//onboard/global:clock",
        "//onboard/global:counter",
        "//onboard/global:singleton",
        "//onboard/lite:lite_callbacks",
        "//onboard/lite:lite_message_util",
        "//onboard/lite/proto:shm_message_cc_proto",
        "//onboard/utils:map_util",
        "@boost//:functional",
        "@boost//:interprocess",
        "@boost//:unordered",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "condition_notifier",
    srcs = ["condition_notifier.cc"],
    hdrs = ["condition_notifier.h"],
    deps = [
        ":shm_manager",
        "//onboard/base",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "lite_shm_header",
    srcs = ["lite_shm_header.cc"],
    hdrs = ["lite_shm_header.h"],
    deps = [
        "//onboard/base",
        "//onboard/global:singleton",
        "//onboard/proto:lite_msg_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "shm_transmitter",
    srcs = ["shm_transmitter.cc"],
    hdrs = ["shm_transmitter.h"],
    deps = [
        ":condition_notifier",
        ":lite_shm_header",
        ":shm_manager",
        "//onboard/global:counter",
        "//onboard/lite:lite_message_util",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "shm_flags",
    srcs = ["shm_flags.cc"],
    hdrs = ["shm_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

######################
# Tests
cc_test(
    name = "shm_manager_test",
    srcs = [
        "shm_manager_test.cc",
    ],
    deps = [
        ":condition_notifier",
        ":lite_shm_header",
        ":shm_manager",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lite_shm_header_test",
    size = "small",
    srcs = ["lite_shm_header_test.cc"],
    deps = [
        ":lite_shm_header",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "shm_transmitter_test",
    srcs = ["shm_transmitter_test.cc"],
    deps = [
        ":condition_notifier",
        ":lite_shm_header",
        ":shm_manager",
        ":shm_transmitter",
        "//onboard/lite/transport/shared_memory:lite_shared_memory_manager_factory",
        "//onboard/lite/transport/shared_memory:lite_shm_shared_memory_manager",
        "@com_google_absl//absl/strings",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "condition_notifier_test",
    srcs = ["condition_notifier_test.cc"],
    deps = [
        ":condition_notifier",
        ":shm_manager",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "shm_debug_util_test",
    srcs = ["shm_debug_util_test.cc"],
    deps = [
        ":shm_debug_util",
        ":shm_manager",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

###############################################################################
# Binaries
cc_binary(
    name = "shm_manager_test_main",
    srcs = [
        "shm_manager_test_main.cc",
    ],
    deps = [
        ":condition_notifier",
        ":shm_manager",
        "//onboard/global:init_qcraft",
        "@boost//:process",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "detroy_shm_main",
    srcs = [
        "detroy_shm_main.cc",
    ],
    deps = [
        ":shm_manager",
        "//onboard/global:ftrace",
    ],
)
