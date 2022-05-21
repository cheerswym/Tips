load("//bazel:common.bzl", "if_cuda")
load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")
load("//bazel:rules_cuda.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "range_image",
    srcs = ["range_image.cc"],
    hdrs = ["range_image.h"],
    deps = [
        ":range_image_util",
        "//onboard/camera/utils:camera_util",
        "//onboard/eval:qevent",
        "//onboard/global:car_common",
        "//onboard/global:run_context",
        "//onboard/global:trace",
        "//onboard/lidar:spin_reader",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/perception:projection_util",
        "//onboard/perception:semantic_segmentation_result",
        "//onboard/proto:perception_cc_proto",
        "@com_google_absl//absl/container:flat_hash_set",
        "@com_google_absl//absl/strings:str_format",
        "@opencv",
    ],
)

cc_library(
    name = "range_image_util",
    srcs = ["range_image_util.cc"],
    hdrs = ["range_image_util.h"],
    deps = [
        "//onboard/lidar:lidar_util",
        "//onboard/params/v2/proto/assembly:vehicle_cc_proto",
        "//onboard/perception:semantic_segmentation_result",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "//onboard/perception/range_image:range_image_dense_crf_cpu",
        "@com_google_absl//absl/container:flat_hash_set",
    ] + if_cuda([
        "//onboard/perception/range_image:range_image_dense_crf_gpu",
    ]),
)

cc_library(
    name = "range_image_dense_crf_cpu",
    srcs = [
        "range_image_dense_crf_base.cc",
        "range_image_dense_crf_cpu.cc",
    ],
    hdrs = [
        "range_image_dense_crf_base.h",
        "range_image_dense_crf_cpu.h",
    ],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/proto:perception_cc_proto",
        "//third_party/dense_crf:dense_crf_cpu",
        "@com_google_absl//absl/container:flat_hash_map",
        "@opencv",
    ],
)

cuda_library(
    name = "range_image_dense_crf_gpu",
    srcs = [
        "range_image_dense_crf_base.cc",
        "range_image_dense_crf_gpu.cc",
    ],
    hdrs = [
        "range_image_dense_crf_base.h",
        "range_image_dense_crf_gpu.h",
    ],
    deps = [
        "//onboard/eval:qevent",
        "//onboard/global:trace",
        "//onboard/proto:perception_cc_proto",
        "//third_party/dense_crf:dense_crf_gpu",
        "@com_google_absl//absl/container:flat_hash_map",
        "@com_google_absl//absl/strings:str_format",
        "@cuda",
        "@opencv",
    ],
)

cc_binary(
    name = "range_image_main",
    srcs = [
        "range_image_main.cc",
    ],
    deps = [
        ":range_image",
        "//offboard/vfs",
        "//onboard/global:init_qcraft",
        "//onboard/lidar:spin_reader_test_util",
        "//onboard/utils:run_util",
    ],
)

cc_binary(
    name = "range_image_bm",
    srcs = [
        "range_image_bm.cc",
    ],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    tags = ["bm"],
    deps = [
        ":range_image",
        "//onboard/lidar:spin_reader_test_util",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_binary(
    name = "range_image_dense_crf_bm",
    srcs = [
        "range_image_dense_crf_bm.cc",
    ],
    tags = ["bm"],
    deps = [
        ":range_image_dense_crf_cpu",
        "@opencv",
        "@com_github_google_benchmark//:benchmark",
    ] + if_cuda([
        ":range_image_dense_crf_gpu",
    ]),
)

cc_test(
    name = "range_image_dense_crf_test",
    srcs = ["range_image_dense_crf_test.cc"],
    deps = [
        ":range_image_dense_crf_cpu",
        "//onboard/proto:perception_cc_proto",
        "@opencv",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        ":range_image_dense_crf_gpu",
    ]),
)

cc_test(
    name = "range_image_dense_crf_sync_test",
    srcs = ["range_image_dense_crf_sync_test.cc"],
    deps = [
        ":range_image_dense_crf_cpu",
        ":range_image_dense_crf_gpu",
        "//onboard/proto:perception_cc_proto",
        "@com_github_google_glog//:glog",
        "@com_google_absl//absl/time",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cpplint()
