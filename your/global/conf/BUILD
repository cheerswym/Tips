load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "conf",
    srcs = ["conf.cc"],
    hdrs = ["conf.h"],
    data = ["//onboard/conf:data"],
    deps = [
        "//onboard/global/conf/proto:configuration_cc_proto",
        "//onboard/lite:check",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/synchronization",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
