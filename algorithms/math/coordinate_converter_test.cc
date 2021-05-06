#include "onboard/math/coordinate_converter.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace {

TEST(CoordinateConverterTest, TestSmoothToGlobal) {
  std::vector<Vec3d> smooth_coords = {
      {0.0, 0.0, 0.0},
      {1.0, 1.0, 1.0},
      {-1.0, -1.0, -1.0},
      {1e5, 1e5, 1e5},
  };
  std::vector<Vec3d> global_coords = {
      {0.0, 0.0, 0.0},
      {1.0, 1.0, 1.0},
      {-1.0, -1.0, -1.0},
  };
  constexpr double kEpsilon = 1e-8;

  CoordinateConverter coordinate_converter(Vec2d(0.0, 0.0));

  const auto test_smooth_to_global = [&] {
    for (const auto& smooth : smooth_coords) {
      const Vec3d global = coordinate_converter.SmoothToGlobal(smooth);
      const Vec3d smooth_2 = coordinate_converter.GlobalToSmooth(global);

      EXPECT_NEAR(smooth.x(), smooth_2.x(), kEpsilon);
      EXPECT_NEAR(smooth.y(), smooth_2.y(), kEpsilon);
      EXPECT_NEAR(smooth.z(), smooth_2.z(), kEpsilon);
    }
  };
  const auto test_global_to_smooth = [&] {
    for (const auto& global : global_coords) {
      const Vec3d smooth = coordinate_converter.GlobalToSmooth(global);
      const Vec3d global_2 = coordinate_converter.SmoothToGlobal(smooth);

      EXPECT_NEAR(global.x(), global_2.x(), kEpsilon);
      EXPECT_NEAR(global.y(), global_2.y(), kEpsilon);
      EXPECT_NEAR(global.z(), global_2.z(), kEpsilon);
    }
  };

  test_smooth_to_global();
  test_global_to_smooth();

  LocalizationTransformProto localization_transform_proto;
  coordinate_converter.UpdateLocalizationTransform(
      localization_transform_proto);
  test_smooth_to_global();
  test_global_to_smooth();

  localization_transform_proto.set_global_ref_x(1.0);
  localization_transform_proto.set_global_ref_y(0.3);
  localization_transform_proto.set_yaw_diff(-1.0);
  coordinate_converter.UpdateLocalizationTransform(
      localization_transform_proto);
  test_smooth_to_global();
  test_global_to_smooth();

  localization_transform_proto.set_global_ref_x(-1.0);
  localization_transform_proto.set_global_ref_y(1.0);
  localization_transform_proto.set_smooth_ref_x(1000);
  localization_transform_proto.set_smooth_ref_y(-2000);
  localization_transform_proto.set_yaw_diff(0.1);
  coordinate_converter.UpdateLocalizationTransform(
      localization_transform_proto);
  test_smooth_to_global();
  test_global_to_smooth();

  CoordinateConverterProto coordinate_converter_proto;
  *(coordinate_converter_proto.mutable_localization_transform_proto()) =
      localization_transform_proto;

  CoordinateConverter coordinate_converter_from_proto;
  coordinate_converter_from_proto.FromProto(coordinate_converter_proto);

  CoordinateConverterProto coordinate_converter_to_proto;
  coordinate_converter_from_proto.ToProto(&coordinate_converter_to_proto);

  const auto test_coordinate_converter_proto = [coordinate_converter_proto,
                                                coordinate_converter_to_proto] {
    EXPECT_TRUE(coordinate_converter_proto.has_localization_transform_proto());
    EXPECT_TRUE(
        coordinate_converter_to_proto.has_localization_transform_proto());

    const auto init_localization_transform_proto =
        coordinate_converter_proto.localization_transform_proto();
    const auto dump_localization_transform_proto =
        coordinate_converter_to_proto.localization_transform_proto();
    EXPECT_EQ(init_localization_transform_proto.timestamp(),
              dump_localization_transform_proto.timestamp());
    EXPECT_EQ(init_localization_transform_proto.smooth_ref_x(),
              dump_localization_transform_proto.smooth_ref_x());
    EXPECT_EQ(init_localization_transform_proto.smooth_ref_y(),
              dump_localization_transform_proto.smooth_ref_y());
    EXPECT_EQ(init_localization_transform_proto.global_ref_x(),
              dump_localization_transform_proto.global_ref_x());
    EXPECT_EQ(init_localization_transform_proto.global_ref_y(),
              dump_localization_transform_proto.global_ref_y());
    EXPECT_EQ(init_localization_transform_proto.yaw_diff(),
              dump_localization_transform_proto.yaw_diff());
    EXPECT_EQ(init_localization_transform_proto.z_diff(),
              dump_localization_transform_proto.z_diff());
  };
  test_coordinate_converter_proto();
}
}  // namespace

}  // namespace qcraft
