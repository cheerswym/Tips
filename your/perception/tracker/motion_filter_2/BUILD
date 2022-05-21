load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracker_common",
    hdrs = ["tracker_common.h"],
    deps = [
        "//onboard/math:eigen",
    ],
)

cc_test(
    name = "tracker_common_test",
    srcs = ["tracker_common_test.cc"],
    deps = [
        ":tracker_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "normal_distribution",
    hdrs = ["normal_distribution.h"],
    deps = [
        ":tracker_common",
    ],
)

#### Motion Model Library ####
cc_library(
    name = "point_model",
    hdrs = ["point_model.h"],
    deps = [
        ":tracker_common",
        "//onboard/math:vec",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_test(
    name = "point_model_test",
    srcs = ["point_model_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":point_model",
        ":tracker_common",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "car_model",
    hdrs = ["car_model.h"],
    deps = [
        ":tracker_common",
        "//onboard/math:vec",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_test(
    name = "car_model_test",
    srcs = ["car_model_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":car_model",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "state_data",
    hdrs = ["state_data.h"],
    deps = [
        ":car_model",
        ":extent_model",
        ":img_2d_model",
        ":point_model",
        ":tracker_common",
    ],
)

cc_test(
    name = "state_data_test",
    srcs = [
        "state_data_test.cc",
    ],
    deps = [
        ":state_data",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

#### Measurement Model Library ####
cc_library(
    name = "meas_model",
    hdrs = ["meas_model.h"],
    deps = [
        ":car_model",
        ":img_2d_model",
        ":point_model",
        ":tracker_common",
        "//onboard/lite:logging",
    ],
)

cc_library(
    name = "meas_model_type_traits",
    hdrs = ["meas_model_type_traits.h"],
    deps = [
        ":extent_meas_model",
        ":meas_model",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "meas_model_test",
    srcs = ["meas_model_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":car_model",
        ":meas_model",
        ":meas_model_type_traits",
        ":point_model",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "meas_model_type_traits_test",
    srcs = ["meas_model_type_traits_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":car_model",
        ":meas_model",
        ":meas_model_type_traits",
        ":point_model",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

#### Filter Library ####

cc_library(
    name = "kalman_filter_base",
    hdrs = ["kalman_filter_base.h"],
    deps = [
        ":tracker_common",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
    ],
)

cc_library(
    name = "kalman_filter",
    hdrs = ["kalman_filter.h"],
    deps = [
        "kalman_filter_base",
        ":meas_model_type_traits",
        ":normal_distribution",
        ":state_data",
        ":tracker_common",
        ":utils",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "kalman_filter_test",
    srcs = ["kalman_filter_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":kalman_filter",
        ":meas_model",
        ":point_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ukf",
    hdrs = ["ukf.h"],
    deps = [
        ":kalman_filter_base",
        ":meas_model_type_traits",
        ":normal_distribution",
        ":state_data",
        ":tracker_common",
        ":utils",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "ukf_test",
    srcs = ["ukf_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":car_model",
        ":meas_model",
        ":ukf",
        "@com_google_googletest//:gtest_main",
    ],
)

#### Estimator Library ####
cc_library(
    name = "motion_filter",
    hdrs = ["motion_filter.h"],
    deps = [
        ":car_model",
        ":kalman_filter",
        ":point_model",
        ":tracker_common",
        ":ukf",
        ":utils",
        "//onboard/eval:qevent",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "motion_filter_test",
    srcs = ["motion_filter_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":motion_filter",
        "//onboard/lite:logging",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "estimator",
    srcs = ["estimator.cc"],
    hdrs = ["estimator.h"],
    deps = [
        ":extent_filter",
        ":motion_filter",
        ":state_data",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "estimator_test",
    srcs = ["estimator_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":estimator",
        ":meas_model",
        "//onboard/lite:logging",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "utils",
    srcs = ["utils.cc"],
    hdrs = ["utils.h"],
    deps = [
        ":extent_meas_model",
        ":meas_model",
        "//onboard/math/geometry:util",
        "//onboard/perception/tracker/motion_filter_2/proto:motion_filter_cc_proto",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "utils_test",
    srcs = ["utils_test.cc"],
    deps = [
        ":car_model",
        ":meas_model",
        ":point_model",
        ":utils",
        "//onboard/lite:logging",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "extent_model",
    hdrs = ["extent_model.h"],
    deps = [
        ":tracker_common",
        "//onboard/math:vec",
        "//onboard/perception/tracker/motion_filter_2/proto:motion_filter_cc_proto",
    ],
)

cc_test(
    name = "extent_model_test",
    srcs = ["extent_model_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":extent_model",
        ":tracker_common",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "extent_meas_model",
    hdrs = ["extent_meas_model.h"],
    deps = [
        ":extent_model",
        ":tracker_common",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "extent_meas_model_test",
    srcs = ["extent_meas_model_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":extent_meas_model",
        ":extent_model",
        ":img_2d_model",
        ":meas_model_type_traits",
        ":tracker_common",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

#### Image 2D tracking Library ####
cc_library(
    name = "img_2d_model",
    hdrs = ["img_2d_model.h"],
    deps = [
        ":tracker_common",
        "//onboard/math:vec",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_test(
    name = "img_2d_model_test",
    srcs = ["img_2d_model_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":extent_meas_model",
        ":extent_model",
        ":img_2d_model",
        ":meas_model_type_traits",
        ":tracker_common",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "extent_filter",
    hdrs = ["extent_filter.h"],
    deps = [
        "kalman_filter_base",
        ":meas_model_type_traits",
        ":state_data",
        ":tracker_common",
        ":utils",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "extent_filter_test",
    srcs = ["extent_filter_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":extent_filter",
        ":extent_model",
        ":meas_model",
        ":tracker_common",
        "//onboard/lite:logging",
        "//onboard/utils:file_util",
        "@com_google_googletest//:gtest_main",
    ],
)

#### Image 2D Kalman Filter Library ####
cc_library(
    name = "img_2d_kalman_filter",
    hdrs = ["img_2d_kalman_filter.h"],
    deps = [
        "kalman_filter_base",
        ":meas_model_type_traits",
        ":normal_distribution",
        ":state_data",
        ":tracker_common",
        ":utils",
        "//onboard/lite:logging",
    ],
)

cc_test(
    name = "img_2d_kalman_filter_test",
    srcs = ["img_2d_kalman_filter_test.cc"],
    data = [
        "//onboard/perception/tracker/motion_filter_2/data:motion_filter_param",
    ],
    deps = [
        ":img_2d_kalman_filter",
        ":img_2d_model",
        ":meas_model",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
