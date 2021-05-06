#ifndef ONBOARD_PLANNER_SMOOTH_LANE_PATH_H_
#define ONBOARD_PLANNER_SMOOTH_LANE_PATH_H_

#include <utility>
#include <vector>

#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/smooth_lane.h"

namespace qcraft {
namespace planner {

class SmoothLanePath {
 public:
  // Function lateral_offset gives the lateral offset (positive for left) at any
  // given arclength along the lane path.
  explicit SmoothLanePath(const SemanticMapManager *semantic_map_manager,
                          mapping::LanePath lane_path,
                          const std::function<double(double)> &lateral_offset =
                              SmoothLane::kNoLateralOffset);

  double s_max() const { return s_max_; }
  Vec3d Sample(double s) const;
  std::vector<Vec3d> Sample(const std::vector<double> &s) const;

  Vec3d SampleTangent(double s) const;
  std::vector<Vec3d> SampleTangent(const std::vector<double> &s) const;

  Vec3d SampleCurvatureNormal(double s) const;
  std::vector<Vec3d> SampleCurvatureNormal(const std::vector<double> &s) const;

  const mapping::LanePath &lane_path() const { return lane_path_; }
  const std::vector<const mapping::LaneProto *> &lanes() const {
    return lanes_;
  }
  const std::vector<SmoothLane> &smooth_lanes() const { return smooth_lanes_; }
  const std::vector<double> &lane_end_s() const { return lane_end_s_; }
  double lane_length(int i) const {
    return i == 0 ? lane_end_s_[0] : lane_end_s_[i] - lane_end_s_[i - 1];
  }
  double path_length() const { return lane_end_s_.back(); }

  mapping::LanePoint GetLanePointForSample(double s) const;

  mapping::LanePath GetPrefixLanePath(double end_s) const;
  mapping::LanePath GetSuffixLanePath(double start_s) const;
  mapping::LanePath GetSubLanePath(double start_s, double end_s) const;

  void SendCanvas() const;

 protected:
  std::pair<int, double> GetLaneIndexAndRelativeSForSample(double s) const;

 private:
  mapping::LanePath lane_path_;
  std::vector<const mapping::LaneProto *> lanes_;
  std::vector<SmoothLane> smooth_lanes_;
  std::vector<double> lane_end_s_;

  double s_max_ = 0.0;
  double start_fraction_ = 0.0;
  double end_fraction_ = 0.0;
  double s0_ = 0.0;

  const SemanticMapManager *semantic_map_manager_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SMOOTH_LANE_PATH_H_
