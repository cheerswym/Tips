#include "onboard/perception/pillar_rc_coord_converter.h"

namespace qcraft {

PillarRCCoordConverter::PillarRCCoordConverter(const double range_front,
                                               const double range_left,
                                               const double diameter)
    : diameter_(diameter),
      diameter_inv_(1.0 / diameter),
      row_offset_(FloorToInt(range_front * diameter_inv_)),
      col_offset_(FloorToInt(range_left * diameter_inv_)) {}

void PillarRCCoordConverter::InitializeWithPose(const VehiclePose &pose) {
  vehicle_to_smooth_transform_ =
      AffineTransformation::FromTranslation(pose.x, pose.y, 0)
          .ApplyYawPitchRoll(pose.yaw, 0, 0);
  smooth_to_vehicle_transform_ = vehicle_to_smooth_transform_.Inverse();
}

}  // namespace qcraft
