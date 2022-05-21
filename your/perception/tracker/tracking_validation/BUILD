load("@rules_python//python:defs.bzl", "py_binary")
load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "visualize_result",
    srcs = [
        ":scripts/visualize_result.py",
    ],
    deps = [
        "//onboard/perception/tracker/tracking_validation/proto:validation_py_pb2",
    ],
)

py_binary(
    name = "visualize_result_imm",
    srcs = [
        ":scripts/visualize_result_imm.py",
    ],
    deps = [
        "//onboard/perception/tracker/tracking_validation/proto:validation_imm_py_pb2",
    ],
)

py_binary(
    name = "visualize_result_extent",
    srcs = [
        ":scripts/visualize_result_extent.py",
    ],
    deps = [
        "//onboard/perception/tracker/tracking_validation/proto:validation_py_pb2",
    ],
)

cpplint()
