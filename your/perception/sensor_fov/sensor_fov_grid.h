#ifndef ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_GRID_H_
#define ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_GRID_H_

#include <atomic>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "onboard/async/async_util.h"
#include "onboard/global/spin_lock.h"
#include "onboard/lite/logging.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/perception/pillar_rc_coord_converter.h"
#include "onboard/perception/sensor_fov/sensor_fov_constants.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::sensor_fov {

struct SFDetectionRange {
  SFDetectionRange(const double front, const double behind, const double left,
                   const double right)
      : front(front), behind(behind), left(left), right(right) {}

  SFDetectionRange() = default;

  explicit SFDetectionRange(const DetectionRangeProto& proto) {
    this->FromProto(proto);
  }

  DetectionRangeProto ToProto() const;
  void FromProto(const DetectionRangeProto& proto);
  Box2d ToBox2d() const;

  double front = 0.0;
  double behind = 0.0;
  double left = 0.0;
  double right = 0.0;
};

struct SFGridInfo {
  void Clear() {
    state = SF_UNOBSERVED;
    max_invisable_elevation = std::numeric_limits<float>::max();
    min_invisable_elevation = 0.f;
  }

  SensorFovState state = SF_UNOBSERVED;
  float max_invisable_elevation = std::numeric_limits<float>::max();
  float min_invisable_elevation = 0.f;
};

struct SFPillar {
  static constexpr int kSerializedSize = 3;  // bytes

  void Clear() {
    state = SF_UNOBSERVED;
    max_invisable_elevation = 0.f;
    min_invisable_elevation = 0.f;
  }
  void Serialize(uint8_t* data) const {
    *data = static_cast<uint8_t>(state);
    *(data + 1) = std::clamp(RoundToInt(max_invisable_elevation *
                                        (1.0 / kSerializedElevationResolution)),
                             0, 255);
    *(data + 2) = std::clamp(RoundToInt(min_invisable_elevation *
                                        (1.0 / kSerializedElevationResolution)),
                             0, 255);
  }
  void Deserialize(const uint8_t* data) {
    state = static_cast<SensorFovState>(*data);
    max_invisable_elevation =
        static_cast<float>(*(data + 1)) * kSerializedElevationResolution;
    min_invisable_elevation =
        static_cast<float>(*(data + 2)) * kSerializedElevationResolution;
  }

  SensorFovState state = SF_UNOBSERVED;
  float max_invisable_elevation = 0.f;
  float min_invisable_elevation = 0.f;
};

/*
                                Grid
     0 ______________________________________________________width
      |                                                      |
      |                                                      |
      |                           x                          |
      |                           ^                          |
      |                           |                          |
      |                           |                          |
      |                           |                          |
      |                           |                          |
      |                           |                          |
      |                           |                          |
      |         y <---------------|----------------          |
      |                           | 0                        |
      |                           |                          |
      |                           |    Vehicle Coordinate    |
      |                           |                          |
      |                           |                          |
      |                                                      |
      |                                                      |
      |______________________________________________________|
height
*/
template <typename InfoType>
class SensorFovGrid {
  friend class SensorFovBuilder;

 public:
  SensorFovGrid(const double range_front, const double range_rear,
                const double range_left, const double range_right,
                const double diameter)
      : width_(CeilToInt((range_left + range_right) / diameter)),
        height_(CeilToInt((range_front + range_rear) / diameter)),
        diameter_(diameter),
        detection_range_(range_front, range_rear, range_left, range_right),
        rc_coord_converter_(range_front, range_left, diameter),
        info_grid_(width_ * height_) {}
  // Not default constructible.
  SensorFovGrid() = delete;
  // User-declared destructor prevents the implicitly-declared move
  // constructor and copy constructor is used every place the move constructor
  // should have been called. So we must explicitly defalut declare it here.
  SensorFovGrid(SensorFovGrid&&) = default;
  // Non-copyable
  SensorFovGrid(const SensorFovGrid&) = delete;

  ~SensorFovGrid() { reset_future_.Wait(); }

  void InitializeWithPose(const VehiclePose& pose) {
    // Wait until the reset is done.
    reset_future_.Wait();
    rc_coord_converter_.InitializeWithPose(pose);
  }

  std::pair<int, int> VehicleCoordToRC(const Vec2d& coord) const {
    return rc_coord_converter_.VehicleCoordToRC(coord);
  }

  Vec2d RCToVehicleCoord(const std::pair<int, int>& rc) const {
    return rc_coord_converter_.RCToVehicleCoord(rc);
  }

  std::pair<int, int> SmoothCoordToRC(const Vec2d& coord) const {
    return rc_coord_converter_.SmoothCoordToRC(coord);
  }

  Vec2d RCToSmoothCoord(const std::pair<int, int>& rc) const {
    return rc_coord_converter_.RCToSmoothCoord(rc);
  }

  bool IsValidRC(const std::pair<int, int>& rc) const {
    return rc.first >= 0 && rc.first < height_ && rc.second >= 0 &&
           rc.second < width_;
  }

  bool IsVehicleCoordInRange(const Vec2d& coord) const {
    const auto rc = VehicleCoordToRC(coord);
    return IsValidRC(rc);
  }

  bool IsSmoothCoordInRange(const Vec2d& coord) const {
    const auto rc = SmoothCoordToRC(coord);
    return IsValidRC(rc);
  }

  InfoType& operator()(const int row, const int col) {
    DCHECK(IsValidRC({row, col}))
        << row << " " << col << " " << height_ << " " << width_;
    return info_grid_[row * width_ + col];
  }
  const InfoType& operator()(const int row, const int col) const {
    DCHECK(IsValidRC({row, col}))
        << row << " " << col << " " << height_ << " " << width_;
    return info_grid_[row * width_ + col];
  }

  void ResetAsync() {
    reset_future_ = ScheduleFuture(ThreadPool::DisposalPool(), [=] {
      SCOPED_QTRACE("SensorFovGrid::ResetAsync");
      for (auto& info : info_grid_) {
        info.Clear();
      }
    });
  }

  const PillarRCCoordConverter& rc_coord_converter() const {
    return rc_coord_converter_;
  }

  int width() const { return width_; }
  int height() const { return height_; }
  double diameter() const { return diameter_; }
  const SFDetectionRange& detection_range() const { return detection_range_; }
  // Serialize info grid to bytes.
  void Serialize(std::string* bytes) const {
    QCHECK_NOTNULL(bytes);
    bytes->resize(InfoType::kSerializedSize * info_grid_.size());
    uint8_t* data = reinterpret_cast<uint8_t*>(bytes->data());
    for (const auto& info : info_grid_) {
      info.Serialize(data);
      data += InfoType::kSerializedSize;
    }
  }
  // Deserialize info grid from bytes.
  void Deserialize(const std::string_view& bytes) {
    QCHECK_EQ(bytes.size(), InfoType::kSerializedSize * info_grid_.size());
    const uint8_t* data = reinterpret_cast<const uint8_t*>(bytes.data());
    for (auto& info : info_grid_) {
      info.Deserialize(data);
      data += InfoType::kSerializedSize;
    }
  }

 private:
  const int width_;
  const int height_;
  const double diameter_;
  SFDetectionRange detection_range_;
  PillarRCCoordConverter rc_coord_converter_;
  std::vector<InfoType> info_grid_;
  // A future used to reset info_grid.
  Future<void> reset_future_;
};

}  // namespace qcraft::sensor_fov

#endif  // ONBOARD_PERCEPTION_SENSOR_FOV_SENSOR_FOV_GRID_H_
