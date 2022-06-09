load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_library(
    name = "qevents_store",
    srcs = ["qevents_store.cc"],
    hdrs = ["qevents_store.h"],
    deps = [
        "//offboard/db/cassandra:cass_table_store",
        "//offboard/eval/proto:qevents_service_cc_grpc",
    ],
)

cc_library(
    name = "stats_store",
    srcs = ["stats_store.cc"],
    hdrs = ["stats_store.h"],
    deps = [
        "//offboard/db/cassandra:cass_table_store",
        "//offboard/eval/proto:qevents_service_cc_grpc",
    ],
)

cpplint()
