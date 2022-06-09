load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
cc_library(
    name = "qevents",
    srcs = [
        "factory.cc",
        "qevents.cc",
    ],
    hdrs = [
        "factory.h",
        "qevents.h",
    ],
    deps = [
        "//offboard/dashboard/services/utils:filter",
        "//offboard/dashboard/services/utils:status",
        "//offboard/db:proto_store",
        "//offboard/db/cassandra:cass_table_store",
        "//offboard/eval/proto:qevents_service_cc_grpc",
        "//offboard/eval/services/qevents/cassandra:qevents_store",
        "//offboard/eval/services/qevents/cassandra:stats_store",
        "//offboard/eval/services/qevents/grouper:qevents_grouper",
        "//onboard/global:clock",
        "//onboard/global:logging",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_grpc_grpc//:grpc++_reflection",
        "@com_google_absl//absl/time",
    ],
)

cpplint()
