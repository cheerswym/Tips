load("//bazel:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "pbq_params",
    data = glob([
        "**",
    ]),
)

cpplint()
