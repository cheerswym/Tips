#ifndef ONBOARD_MATH_COORDINATE_CONVERTER_H_
#define ONBOARD_MATH_COORDINATE_CONVERTER_H_

#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/check.h"
#include "onboard/maps/imagery_types.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/util.h"
#include "onboard/proto/localization.pb.h"

namespace qcraft {

// A class for converting between global/smooth/local coordinates.
class CoordinateConverter {
 public:
  // NOTE: return a coordinate converter that treat the legacy local coordinate
  // as the smooth coordinate, only for backward compatibility.
  static CoordinateConverter FromLocale();

  // The argument `correct_yaw' will be used to correct the yaw from
  // localization, which is a temporary solution and will be deprecated after
  // the lidar yaw calibration issue is resolved.
  static CoordinateConverter FromLocalizationTransform(
      const LocalizationTransformProto& localization_transform_proto,
      bool correct_yaw = false);

  CoordinateConverter() {}

  // Create a CoordinateConverter object with the given global point as origin.
  explicit CoordinateConverter(Vec2d origin_global) {
    SetSmoothCoordinateOrigin(origin_global);
  }

  // Create a CoordinateConverter object from localization result.
  explicit CoordinateConverter(
      const LocalizationTransformProto& localization_transform_proto)
      : CoordinateConverter(
            FromLocalizationTransform(localization_transform_proto)) {}

  void FromProto(const CoordinateConverterProto& proto) {
    *this = FromLocalizationTransform(proto.localization_transform_proto());
  }

  void ToProto(CoordinateConverterProto* proto) const {
    *(proto->mutable_localization_transform_proto()) = localization_transform_;
  }

  // Set the given <longitude, latitude> as the origin of the smooth coordinate,
  // and assume there is no rotation between smooth and global coordinates.
  void SetSmoothCoordinateOrigin(Vec2d global_coord);

  // Build transformation between smooth and global coordinate using the given
  // localization result.
  // [[deprecated("use create FromLocalizationTransform()  method instead ")]]
  void UpdateLocalizationTransform(
      const LocalizationTransformProto& localization_transform_proto) {
    *this = FromLocalizationTransform(localization_transform_proto);
  }

  void UpdateLocalizationTransform(const Vec2d& smooth_ref,
                                   const Vec2d& global_ref, double z_diff = 0.0,
                                   double yaw_diff = 0.0) {
    LocalizationTransformProto localization_transform;
    localization_transform.set_smooth_ref_x(smooth_ref.x());
    localization_transform.set_smooth_ref_y(smooth_ref.y());
    localization_transform.set_global_ref_x(global_ref.x());
    localization_transform.set_global_ref_y(global_ref.y());
    localization_transform.set_z_diff(z_diff);
    localization_transform.set_yaw_diff(yaw_diff);
    UpdateLocalizationTransform(localization_transform);
  }

  ImageryLevelId GetLevel() const {
    return localization_transform_.has_level_id()
               ? localization_transform_.level_id()
               : 0;
  }

  double SmoothYawToGlobal(double smooth_yaw) const {
    return NormalizeAngle(smooth_yaw + localization_transform_.yaw_diff());
  }

  double SmoothYawToGlobalNoNormalize(double smooth_yaw) const {
    return smooth_yaw + localization_transform_.yaw_diff();
  }

  double GlobalYawToSmooth(double global_yaw) const {
    return NormalizeAngle(global_yaw - localization_transform_.yaw_diff());
  }

  double GlobalYawToSmoothNoNormalize(double global_yaw) const {
    return global_yaw - localization_transform_.yaw_diff();
  }

  Vec2d SmoothToGlobal(const Vec2d& pos_smooth) const {
    const auto& coeffs = smooth_to_global_coeffs_;
    return {
        pos_smooth.x() * coeffs[0] + pos_smooth.y() * coeffs[1] + coeffs[2],
        pos_smooth.x() * coeffs[3] + pos_smooth.y() * coeffs[4] + coeffs[5]};
  }

  Vec2d GlobalToSmooth(const Vec2d& pos_global) const {
    const auto& coeffs = global_to_smooth_coeffs_;
    return {
        pos_global.x() * coeffs[0] + pos_global.y() * coeffs[1] + coeffs[2],
        pos_global.x() * coeffs[3] + pos_global.y() * coeffs[4] + coeffs[5]};
  }

  Vec3d SmoothToGlobal(const Vec3d& pos_smooth) const {
    return {SmoothToGlobal({pos_smooth.x(), pos_smooth.y()}),
            pos_smooth.z() + localization_transform_.z_diff()};
  }

  Vec3d GlobalToSmooth(const Vec3d& pos_global) const {
    return {GlobalToSmooth({pos_global.x(), pos_global.y()}),
            pos_global.z() - localization_transform_.z_diff()};
  }

  Vec2d SmoothToGlobalRotate(const Vec2d& pos_smooth) const {
    return pos_smooth.Rotate(localization_transform_.yaw_diff());
  }

  Vec2d GlobalToSmoothRotate(const Vec2d& pos_global) const {
    return pos_global.Rotate(-localization_transform_.yaw_diff());
  }

  Vec2d SmoothToLocalRotate(const Vec2d& pos_smooth) const {
    return pos_smooth.Rotate(localization_transform_.yaw_diff());
  }

  Vec2d LocalToSmoothRotate(const Vec2d& pos_local) const {
    return pos_local.Rotate(-localization_transform_.yaw_diff());
  }

  double LocalToSmoothRotate(double angle) const {
    return angle - localization_transform_.yaw_diff();
  }

  const LocalizationTransformProto& localization_transform() const {
    return localization_transform_;
  }

  const double* global_to_smooth_coeffs() const {
    return global_to_smooth_coeffs_;
  }

 private:
  LocalizationTransformProto localization_transform_;
  // To convert 2d smooth coord into global.
  double smooth_to_global_coeffs_[6];
  // To convert 2d global coord into smooth.
  double global_to_smooth_coeffs_[6];
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_COORDINATE_CONVERTER_H_
