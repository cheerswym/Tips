load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.

cc_library(
    name = "future",
    hdrs = ["future.h"],
)

cc_library(
    name = "concurrent_queue",
    hdrs = ["concurrent_queue.h"],
    deps = [
        "//onboard/lite:check",
    ],
)

cc_library(
    name = "async_util",
    hdrs = ["async_util.h"],
    deps = [
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "async_macro",
    hdrs = ["async_macro.h"],
    deps = [
        ":async_util",
        "//onboard/global:trace",
    ],
)

cc_library(
    name = "thread_pool",
    srcs = ["thread_pool.cc"],
    hdrs = ["thread_pool.h"],
    deps = [
        ":future",
        "//onboard/global:counter",
        "//onboard/lite:check",
        "//onboard/utils:thread_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//absl/base:core_headers",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_library(
    name = "parallel_for",
    srcs = ["parallel_for.cc"],
    hdrs = ["parallel_for.h"],
    deps = [
        "//onboard/async:thread_pool",
        "//onboard/global:trace",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
    ],
)

cc_test(
    name = "async_util_test",
    srcs = ["async_util_test.cc"],
    deps = [
        ":async_util",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "concurrent_queue_test",
    srcs = ["concurrent_queue_test.cc"],
    deps = [
        ":concurrent_queue",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "thread_pool_test",
    srcs = ["thread_pool_test.cc"],
    deps = [
        ":thread_pool",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "parallel_for_test",
    srcs = ["parallel_for_test.cc"],
    deps = [
        ":parallel_for",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "async_macro_test",
    srcs = ["async_macro_test.cc"],
    deps = [
        ":async_macro",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
