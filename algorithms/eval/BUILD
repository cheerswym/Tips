load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
cc_binary(
    name = "qevents_server",
    srcs = ["qevents_server_main.cc"],
    deps = [
        "//offboard/eval/services/qevents",
        "//onboard/global:init_qcraft",
        "//onboard/global:logging",
        "//onboard/proto:port_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

filegroup(
    name = "dataset",
    data = glob([
        "dataset/**",
    ]),
)

cpplint()
