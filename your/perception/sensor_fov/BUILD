load("//bazel:cpplint.bzl", "cpplint")
load("//bazel:rules_cc.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "line_iterator",
    srcs = ["line_iterator.cc"],
    hdrs = ["line_iterator.h"],
    deps = [
        "//onboard/lite:logging",
        "//onboard/math:vec",
    ],
)

cc_test(
    name = "line_iterator_test",
    srcs = ["line_iterator_test.cc"],
    deps = [
        ":line_iterator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sensor_fov_constants",
    hdrs = ["sensor_fov_constants.h"],
)

cc_library(
    name = "sensor_fov_grid",
    srcs = ["sensor_fov_grid.cc"],
    hdrs = ["sensor_fov_grid.h"],
    deps = [
        ":sensor_fov_constants",
        "//onboard/async:async_util",
        "//onboard/global:spin_lock",
        "//onboard/lite:logging",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/math/geometry:box2d",
        "//onboard/perception:pillar_rc_coord_converter",
        "//onboard/proto:perception_cc_proto",
    ],
)

cc_binary(
    name = "sensor_fov_grid_bm",
    testonly = True,
    srcs = ["sensor_fov_grid_bm.cc"],
    tags = ["bm"],
    deps = [
        ":sensor_fov_grid",
        ":sensor_fov_test_util",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_test(
    name = "sensor_fov_grid_test",
    srcs = ["sensor_fov_grid_test.cc"],
    deps = [
        ":sensor_fov_grid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sensor_fov",
    srcs = ["sensor_fov.cc"],
    hdrs = ["sensor_fov.h"],
    deps = [
        ":sensor_fov_constants",
        ":sensor_fov_grid",
        "//onboard/global:trace",
        "//onboard/lidar:vehicle_pose",
        "//onboard/lite:logging",
        "//onboard/math/geometry:affine_transformation",
        "//onboard/math/geometry:box2d",
        "//onboard/math/geometry:polygon2d",
        "//onboard/perception:geometry_util",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_snappy//:snappy",
        "@com_google_absl//absl/status:statusor",
    ],
)

cc_test(
    name = "sensor_fov_test",
    srcs = ["sensor_fov_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":sensor_fov",
        ":sensor_fov_test_util",
        "//onboard/params:param_manager",
        "//onboard/perception:geometry_util",
        "//onboard/perception/test_util:obstacle_builder",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "sensor_fov_bm",
    testonly = True,
    srcs = ["sensor_fov_bm.cc"],
    tags = ["bm"],
    deps = [
        ":sensor_fov",
        ":sensor_fov_test_util",
        "//onboard/perception/test_util:obstacle_builder",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_library(
    name = "sensor_fov_builder",
    srcs = ["sensor_fov_builder.cc"],
    hdrs = ["sensor_fov_builder.h"],
    deps = [
        ":line_iterator",
        ":sensor_fov",
        ":sensor_fov_constants",
        "//offboard/vis/vantage/vantage_server:vantage_client_man",
        "//onboard/async:parallel_for",
        "//onboard/async:thread_pool",
        "//onboard/camera:camera_params",
        "//onboard/global:trace",
        "//onboard/lidar:spin_structs",
        "//onboard/lidar:vehicle_pose",
        "//onboard/math:util",
        "//onboard/math:vec",
        "//onboard/params:param_manager",
        "//onboard/perception:cluster",
        "//onboard/perception:obstacle_util",
        "//onboard/perception/sensor_fov:sensor_fov_grid",
        "//onboard/proto:perception_cc_proto",
        "//onboard/utils:map_util",
        "@com_github_google_snappy//:snappy",
        "@com_google_absl//absl/container:flat_hash_map",
        "@opencv",
    ],
)

cc_test(
    name = "sensor_fov_builder_test",
    srcs = ["sensor_fov_builder_test.cc"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":sensor_fov_builder",
        ":sensor_fov_test_util",
        "//onboard/params:param_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "sensor_fov_builder_bm",
    testonly = True,
    srcs = ["sensor_fov_builder_bm.cc"],
    tags = ["bm"],
    deps = [
        ":sensor_fov_builder",
        ":sensor_fov_test_util",
        "//onboard/perception/test_util:obstacle_builder",
        "@com_github_google_benchmark//:benchmark",
    ],
)

cc_library(
    name = "sensor_fov_test_util",
    testonly = True,
    srcs = ["sensor_fov_test_util.cc"],
    hdrs = ["sensor_fov_test_util.h"],
    data = [
        "//onboard/params/run_params:run_param_data",
    ],
    deps = [
        ":sensor_fov_builder",
        "//onboard/lidar:vehicle_pose",
        "//onboard/params:param_manager",
        "//onboard/perception:cluster",
        "//onboard/perception/test_util:cluster_builder",
    ],
)

cpplint()
