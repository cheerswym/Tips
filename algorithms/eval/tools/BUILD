load("@qcraft_py3_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "qevent_loader",
    srcs = ["qevent_loader.py"],
    deps = [
        "//offboard/eval/proto:qevents_service_py_pb2",
        "//offboard/eval/proto:qevents_service_py_pb2_grpc",
        "//onboard/eval/proto:qevent_py_pb2",
    ],
)

py_binary(
    name = "qevent_query",
    srcs = ["qevent_query.py"],
    deps = [
        requirement("pymongo"),
    ],
)

py_binary(
    name = "qevent_converter",
    srcs = ["qevent_converter.py"],
    deps = [
        "//offboard/simulation/simulation_test/proto:sim_snippet_py_pb2",
    ],
)

py_binary(
    name = "qevent_field_converter",
    srcs = ["qevent_field_converter.py"],
    deps = [
        ":qevent_loader",
        "//offboard/simulation/simulation_test/proto:sim_snippet_py_pb2",
    ],
)

py_binary(
    name = "query_by_qevent_field",
    srcs = ["query_by_qevent_field.py"],
)
