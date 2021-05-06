#include "onboard/math/coordinate_converter.h"

#include "onboard/lite/logging.h"
#include "onboard/maps/map_selector.h"

namespace qcraft {
namespace {
constexpr double kMeterPerRadianOnEquator = 6371000.0;

Vec2d GetOriginCoordFromLocale(Locale locale) {
  const auto locale_info = locale_util::GetLocaleInfo(locale);
  return {locale_info.longitude(), locale_info.latitude()};
}

}  // namespace

CoordinateConverter CoordinateConverter::FromLocale() {
  // For legacy reason, we treat local coordinate as smooth before smooth is
  // introduced. In this case, we need to compute localization_transform_ so
  // that it can correctly convert local coordinate into global.
  CoordinateConverter coordinate_converter;
  coordinate_converter.SetSmoothCoordinateOrigin(
      GetOriginCoordFromLocale(locale_util::GetLocale()));
  return coordinate_converter;
}

void CoordinateConverter::SetSmoothCoordinateOrigin(Vec2d global_coord) {
  const double global_to_smooth_scale_x =
      kMeterPerRadianOnEquator * std::cos(global_coord.y());
  const double global_to_smooth_scale_y = kMeterPerRadianOnEquator;
  const double smooth_to_global_scale_x = 1.0 / global_to_smooth_scale_x;
  const double smooth_to_global_scale_y = 1.0 / global_to_smooth_scale_y;

  smooth_to_global_coeffs_[0] = smooth_to_global_scale_x;
  smooth_to_global_coeffs_[1] = 0.0;
  smooth_to_global_coeffs_[2] = global_coord.x();
  smooth_to_global_coeffs_[3] = 0.0;
  smooth_to_global_coeffs_[4] = smooth_to_global_scale_y;
  smooth_to_global_coeffs_[5] = global_coord.y();

  global_to_smooth_coeffs_[0] = global_to_smooth_scale_x;
  global_to_smooth_coeffs_[1] = 0.0;
  global_to_smooth_coeffs_[2] = -global_to_smooth_scale_x * global_coord.x();
  global_to_smooth_coeffs_[3] = 0.0;
  global_to_smooth_coeffs_[4] = global_to_smooth_scale_y;
  global_to_smooth_coeffs_[5] = -global_to_smooth_scale_y * global_coord.y();
}

CoordinateConverter CoordinateConverter::FromLocalizationTransform(
    const LocalizationTransformProto& localization_transform_proto,
    bool correct_yaw) {
  CoordinateConverter coordinate_converter;
  coordinate_converter.localization_transform_ = localization_transform_proto;

  // To convert a smooth coord
  //   (x, y, z, yaw)
  // into global coord
  //   (lon, lat, alt, yaw_global)
  //
  // lon = coeffs_0 * x + coeffs_1 * y + coeffs_2
  // lat = coeffs_3 * x + coeffs_4 * y + coeffs_5
  // alt = z + z_diff
  // yaw_global = yaw + yaw_diff
  //
  // Now let's compute those coefficients.
  //
  // To convert smooth to global, we have
  //
  // global = (smooth - smooth_ref).Rotate(yaw_diff) / scale + gloabl_ref
  //
  // Then we have
  //
  // global_x = ((sx-sx_ref)*cos_yaw-(sy-sy_ref)*sin_yaw)/scale_x+gx_ref
  //          = sx * cos_yaw / scale_x - sy * sin_yaw / scale_x +
  //            (-sx_ref * cos_yaw + sy_ref * sin_yaw) / scale_x + gx_ref
  // Hence
  //
  // coeffs_0 = cos_yaw / scale_x
  // coeffs_1 = -sin_yaw / scale_x
  // coeffs_2 = (-sx_ref * cos_yaw + sy_ref * sin_yaw) / scale_x + gx_ref
  //
  // global_y = ((sx-sx_ref)*sin_yaw+(sy-sy_ref)*cos_yaw)/scale_y+gy_ref
  //          = sx * sin_yaw / scale_y + sy * cos_yaw / scale_y +
  //            (-sx_ref * sin_yaw - sy_ref * cos_yaw) / scale_y + gy_ref
  // Hence
  //
  // coeffs_3 = sin_yaw / scale_y
  // coeffs_4 = cos_yaw / scale_y
  // coeffs_5 = (-sx_ref * sin_yaw - sy_ref * cos_yaw) / scale_y + gy_ref
  //
  // Similarly
  //
  // smooth = ((global - global_ref) * scale).Rotate(-yaw_diff) + smooth_ref
  //
  // smooth_x = (gx-gx_ref)*scale_x*cos_yaw+(gy-gy_ref)*scale_y*sin_yaw+sx_ref
  //          = gx * cos_yaw * scale_x + gy * sin_yaw * scale_y -
  //            gx_ref * cos_yaw * scale_x - gy_ref * sin_yaw * scale_y + sx_ref
  // Hence
  //
  // coeffs_0 = cos_yaw * scale_x
  // coeffs_1 = sin_yaw * scale_y
  // coeffs_2 = -gx_ref * cos_yaw * scale_x -gy_ref * sin_yaw * scale_y + sx_ref
  //
  // smooth_y = (gx-gx_ref)*scale_x*-sin_yaw+(gy-gy_ref)*scale_y*cos_yaw+sy_ref
  //          = -gx * sin_yaw * scale_x + gy * cos_yaw * scale_y +
  //            gx_ref * sin_yaw * scale_x - gy_ref * cos_yaw * scale_y + sy_ref
  // Hence
  //
  // coeffs_3 = -sin_yaw * scale_x
  // coeffs_4 = cos_yaw * scale_y
  // coeffs_5 = gx_ref * sin_yaw * scale_x - gy_ref * cos_yaw * scale_y + sy_ref

  const double smooth_ref_x = localization_transform_proto.smooth_ref_x();
  const double smooth_ref_y = localization_transform_proto.smooth_ref_y();
  const double yaw_diff =
      localization_transform_proto.yaw_diff() +
      (correct_yaw ? localization_transform_proto.yaw_correct() : 0.0);
  const double global_ref_x = localization_transform_proto.global_ref_x();
  const double global_ref_y = localization_transform_proto.global_ref_y();

  const double sin_yaw = std::sin(yaw_diff);
  const double cos_yaw = std::cos(yaw_diff);
  const double scale_x = kMeterPerRadianOnEquator * std::cos(global_ref_y);
  const double scale_y = kMeterPerRadianOnEquator;
  const double scale_x_inv = 1.0 / scale_x;
  const double scale_y_inv = 1.0 / scale_y;

  coordinate_converter.smooth_to_global_coeffs_[0] = cos_yaw * scale_x_inv;
  coordinate_converter.smooth_to_global_coeffs_[1] = -(sin_yaw * scale_x_inv);
  coordinate_converter.smooth_to_global_coeffs_[2] =
      -smooth_ref_x * (cos_yaw * scale_x_inv) +
      smooth_ref_y * (sin_yaw * scale_x_inv) + global_ref_x;

  coordinate_converter.smooth_to_global_coeffs_[3] = sin_yaw * scale_y_inv;
  coordinate_converter.smooth_to_global_coeffs_[4] = cos_yaw * scale_y_inv;
  coordinate_converter.smooth_to_global_coeffs_[5] =
      -smooth_ref_x * (sin_yaw * scale_y_inv) -
      smooth_ref_y * (cos_yaw * scale_y_inv) + global_ref_y;

  coordinate_converter.global_to_smooth_coeffs_[0] = cos_yaw * scale_x;
  coordinate_converter.global_to_smooth_coeffs_[1] = sin_yaw * scale_y;
  coordinate_converter.global_to_smooth_coeffs_[2] =
      -global_ref_x * (cos_yaw * scale_x) - global_ref_y * (sin_yaw * scale_y) +
      smooth_ref_x;

  coordinate_converter.global_to_smooth_coeffs_[3] = -(sin_yaw * scale_x);
  coordinate_converter.global_to_smooth_coeffs_[4] = cos_yaw * scale_y;
  coordinate_converter.global_to_smooth_coeffs_[5] =
      global_ref_x * (sin_yaw * scale_x) - global_ref_y * (cos_yaw * scale_y) +
      smooth_ref_y;
  return coordinate_converter;
}

}  // namespace qcraft
