load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

qt_cc_library(
    name = "asm_opencv",
    src = "asm_opencv.cc",
    hdr = "asm_opencv.h",
    deps = [
        "@opencv",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)

qt_cc_library(
    name = "imshow",
    src = "imshow.cc",
    hdr = "imshow.h",
    deps = [
        ":asm_opencv",
        "//offboard/calibration/calibrator:zoom_view",
        "//onboard/base",
        "//onboard/lite:logging",
        "//onboard/utils:keyboard_scanner",
        "@com_github_gflags_gflags//:gflags",
        "@opencv",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
    ],
)

cpplint()
