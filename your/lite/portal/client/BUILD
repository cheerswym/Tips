load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "portal_service_client",
    srcs = ["portal_service_client.cc"],
    hdrs = ["portal_service_client.h"],
    deps = [
        "//onboard/lite/portal/proto:portal_service_cc_grpc",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/status:statusor",
        "@com_google_protobuf//:protobuf",
    ],
)

###############################################################################
# Binaries.
cc_binary(
    name = "portal_service_client_main",
    srcs = ["portal_service_client_main.cc"],
    copts = [
        "-fPIC",
    ],
    deps = [
        ":portal_service_client",
        "//onboard/global:init_qcraft",
        "//onboard/proto:port_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//absl/strings",
    ],
)

###############################################################################
# Tests.

cpplint()
