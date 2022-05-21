#ifndef ONBOARD_PERCEPTION_GROUND_REMOVAL_GROUND_REMOVER_H_
#define ONBOARD_PERCEPTION_GROUND_REMOVAL_GROUND_REMOVER_H_

#include <array>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "onboard/lidar/spin_structs.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/assembly/vehicle.pb.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::ground_removal {

class GroundPointTable {
 public:
  GroundPointTable(const int num_scans, const int num_beams)
      : num_scans_(num_scans),
        num_beams_(num_beams),
        table_(num_scans * num_beams, {false, false}) {}

  GroundPointTable() = default;

  bool operator()(const int scan_index, const int beam_index,
                  const int return_index) const {
    QCHECK(scan_index * num_beams_ + beam_index < table_.size());
    return table_[scan_index * num_beams_ + beam_index][return_index];
  }
  bool& operator()(const int scan_index, const int beam_index,
                   const int return_index) {
    QCHECK(scan_index * num_beams_ + beam_index < table_.size());
    return table_[scan_index * num_beams_ + beam_index][return_index];
  }

  int num_scans() const { return num_scans_; }
  int num_beams() const { return num_beams_; }

 private:
  int num_scans_;
  int num_beams_;
  std::vector<std::array<bool, LaserShot::kMaxNumReturns>> table_;
};

using GroundPointTables = absl::flat_hash_map<LidarId, GroundPointTable>;

class GroundRemover {
 public:
  GroundRemover(const std::vector<LidarFrame>& lidar_frames,
                const RunParamsProtoV2& run_params);

  virtual ~GroundRemover() = default;

  virtual void Compute() = 0;

  const GroundPointTables& ground_point_tables() const {
    return ground_point_tables_;
  }

 protected:
  absl::flat_hash_map<LidarId, LidarFrame> lidar_frames_;
  absl::flat_hash_map<LidarId, LidarParametersProto> lidar_params_;
  GroundPointTables ground_point_tables_;
};

}  // namespace qcraft::ground_removal

#endif  // ONBOARD_PERCEPTION_GROUND_REMOVAL_GROUND_REMOVER_H_
