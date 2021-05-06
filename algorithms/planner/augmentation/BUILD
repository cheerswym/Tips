load("@rules_python//python:defs.bzl", "py_binary")
load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

###############################################################################
# Libraries.
py_binary(
    name = "generate_scenario",
    srcs = [
        "generate_scenario.py",
    ],
)

py_binary(
    name = "generate_scenario_array",
    srcs = [
        "generate_scenario_array.py",
    ],
    deps = [
        ":generate_scenario",
        "//onboard/planner/augmentation/proto:augmentation_config_py_pb2",
    ],
)

###############################################################################
# Binaries.

###############################################################################
# Tests.

cpplint()
