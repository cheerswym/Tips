load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "kalman_filter_validation",
    srcs = ["kalman_filter_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:kalman_filter",
        "//onboard/perception/tracker/motion_filter_2:meas_model",
        "//onboard/perception/tracker/motion_filter_2:point_model",
        "//onboard/perception/tracker/tracking_validation/proto:validation_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "ukf_validation",
    srcs = ["ukf_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:car_model",
        "//onboard/perception/tracker/motion_filter_2:meas_model",
        "//onboard/perception/tracker/motion_filter_2:ukf",
        "//onboard/perception/tracker/tracking_validation/proto:validation_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "imm_kf_validation",
    srcs = ["imm_kf_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:estimator",
        "//onboard/perception/tracker/tracking_validation/proto:validation_imm_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "imm_ukf_validation",
    srcs = ["imm_ukf_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:estimator",
        "//onboard/perception/tracker/tracking_validation/proto:validation_imm_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "imm_1m_ukf_validation",
    srcs = ["imm_1m_ukf_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:estimator",
        "//onboard/perception/tracker/tracking_validation/proto:validation_imm_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "extent_filter_validation",
    srcs = ["extent_filter_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:extent_filter",
        "//onboard/perception/tracker/tracking_validation/proto:validation_cc_proto",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cc_binary(
    name = "front_rear_ref_point_validation",
    srcs = ["front_rear_ref_point_validation.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        "//onboard/lite:logging",
        "//onboard/perception/tracker/motion_filter_2:estimator",
        "//onboard/utils:file_util",
        "@com_github_google_glog//:glog",
    ],
)

cpplint()
