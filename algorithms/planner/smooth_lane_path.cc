#include "onboard/planner/smooth_lane_path.h"

#include <algorithm>

#include "absl/strings/str_join.h"
#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/util.h"
#include "onboard/utils/file_util.h"
#include "onboard/vis/common/colormap.h"

namespace qcraft {
namespace planner {

SmoothLanePath::SmoothLanePath(
    const SemanticMapManager *semantic_map_manager, mapping::LanePath lane_path,
    const std::function<double(double)> &lateral_offset)
    : lane_path_(std::move(lane_path)),
      semantic_map_manager_(semantic_map_manager) {
  const int n = lane_path_.lane_ids().size();
  QCHECK_GE(n, 1) << "Lane path has zero lanes.";

  start_fraction_ = lane_path.start_fraction();
  end_fraction_ = lane_path.end_fraction();

  for (const auto &lane : lane_path_) {
    lanes_.push_back(&semantic_map_manager->FindLaneByIdOrDie(lane.lane_id));
  }

  if (n == 1) {
    smooth_lanes_.emplace_back(*semantic_map_manager, *lanes_.front(), nullptr,
                               nullptr, 0.0, lateral_offset);
    QCHECK_LE(start_fraction_, end_fraction_);
    s0_ = smooth_lanes_.front().s_max() * start_fraction_;
    s_max_ = smooth_lanes_.front().s_max() * (end_fraction_ - start_fraction_);
    lane_end_s_.push_back(s_max_);
  } else {
    const double s0 =
        semantic_map_manager->GetLaneLengthOrDie(lane_path_.front().lane_id()) *
        start_fraction_;
    smooth_lanes_.emplace_back(
        *semantic_map_manager, *lanes_.front(), nullptr, lanes_[1], 0.0,
        [&lateral_offset, s0](double s) { return lateral_offset(s - s0); });
    s0_ = s0;
    s_max_ = smooth_lanes_.front().s_max() - s0_;
    lane_end_s_.push_back(s_max_);
    for (int i = 1; i < n; ++i) {
      const auto &prev_smooth_lane = smooth_lanes_.back();
      const double sampling_start_offset =
          SmoothLane::kLaneSampleInterval -
          (prev_smooth_lane.s_max() - prev_smooth_lane.last_sample_s());
      QCHECK_GE(sampling_start_offset, 0.0);
      QCHECK_LE(sampling_start_offset, SmoothLane::kLaneSampleInterval);
      const double s_max_i = s_max_;
      smooth_lanes_.emplace_back(
          *semantic_map_manager, *lanes_[i], lanes_[i - 1],
          i + 1 < n ? lanes_[i + 1] : nullptr, sampling_start_offset,
          [&lateral_offset, s_max_i](double s) {
            return lateral_offset(s + s_max_i);
          });
      s_max_ +=
          smooth_lanes_.back().s_max() * (i + 1 < n ? 1.0 : end_fraction_);
      lane_end_s_.push_back(s_max_);
    }
  }
}

Vec3d SmoothLanePath::Sample(double s) const {
  int index;
  double rel_s;
  std::tie(index, rel_s) = GetLaneIndexAndRelativeSForSample(s);
  return smooth_lanes_[index].Sample(rel_s);
}

std::vector<Vec3d> SmoothLanePath::Sample(const std::vector<double> &s) const {
  // TODO(Fang) avoid binary search at every iteration by incrementing s.
  std::vector<Vec3d> points;
  points.reserve(s.size());
  for (const double s_value : s) points.push_back(Sample(s_value));
  return points;
}

Vec3d SmoothLanePath::SampleTangent(double s) const {
  int index;
  double rel_s;
  std::tie(index, rel_s) = GetLaneIndexAndRelativeSForSample(s);
  return smooth_lanes_[index].SampleTangent(rel_s);
}

std::vector<Vec3d> SmoothLanePath::SampleTangent(
    const std::vector<double> &s) const {
  // TODO(Fang) avoid binary search at every iteration by incrementing s.
  std::vector<Vec3d> points;
  points.reserve(s.size());
  for (const double s_value : s) points.push_back(SampleTangent(s_value));
  return points;
}

Vec3d SmoothLanePath::SampleCurvatureNormal(double s) const {
  int index;
  double rel_s;
  std::tie(index, rel_s) = GetLaneIndexAndRelativeSForSample(s);
  return smooth_lanes_[index].SampleCurvatureNormal(rel_s);
}

std::vector<Vec3d> SmoothLanePath::SampleCurvatureNormal(
    const std::vector<double> &s) const {
  // TODO(Fang) avoid binary search at every iteration by incrementing s.
  std::vector<Vec3d> points;
  points.reserve(s.size());
  for (const double s_value : s) {
    points.push_back(SampleCurvatureNormal(s_value));
  }
  return points;
}

std::pair<int, double> SmoothLanePath::GetLaneIndexAndRelativeSForSample(
    double s) const {
  QCHECK(!lane_end_s_.empty());
  if (s >= s_max_) {
    const int index = lane_end_s_.size() - 1;
    const double lane_relative_s =
        (index == 0 ? s + s0_ : s - lane_end_s_[index - 1]);
    return {index, std::min(lane_relative_s, smooth_lanes_[index].s_max())};
  }
  if (s <= 0.0) return {0, s0_};
  // index in [0, size).
  const int index =
      std::upper_bound(lane_end_s_.begin(), lane_end_s_.end(), s) -
      lane_end_s_.begin();
  QCHECK_GE(index, 0);
  QCHECK_LT(index, lane_end_s_.size());
  const double lane_relative_s =
      (index == 0 ? s + s0_ : s - lane_end_s_[index - 1]);
  return {index, std::min(lane_relative_s, smooth_lanes_[index].s_max())};
}

mapping::LanePoint SmoothLanePath::GetLanePointForSample(double s) const {
  int index;
  double rel_s;
  std::tie(index, rel_s) = GetLaneIndexAndRelativeSForSample(s);
  return {lane_path_.lane_ids()[index], rel_s / smooth_lanes_[index].s_max()};
}

mapping::LanePath SmoothLanePath::GetPrefixLanePath(double end_s) const {
  int index;
  double rel_s;
  std::tie(index, rel_s) = GetLaneIndexAndRelativeSForSample(end_s);
  std::vector<mapping::ElementId> lane_ids;
  lane_ids.reserve(index + 1);
  for (int i = 0; i <= index; ++i) {
    lane_ids.push_back(lane_path().lane_ids()[i]);
  }
  return {semantic_map_manager_, std::move(lane_ids), start_fraction_,
          rel_s / smooth_lanes_[index].s_max()};
}

mapping::LanePath SmoothLanePath::GetSuffixLanePath(double start_s) const {
  int index;
  double rel_s;
  std::tie(index, rel_s) = GetLaneIndexAndRelativeSForSample(start_s);
  std::vector<mapping::ElementId> lane_ids;
  lane_ids.reserve(lane_path().lane_ids().size() - index);
  for (int i = index; i < lane_path().lane_ids().size(); ++i) {
    lane_ids.push_back(lane_path().lane_ids()[i]);
  }
  return {semantic_map_manager_, std::move(lane_ids),
          rel_s / smooth_lanes_[index].s_max(), end_fraction_};
}

mapping::LanePath SmoothLanePath::GetSubLanePath(double start_s,
                                                 double end_s) const {
  int index0, index1;
  double rel_s0, rel_s1;
  std::tie(index0, rel_s0) = GetLaneIndexAndRelativeSForSample(start_s);
  std::tie(index1, rel_s1) = GetLaneIndexAndRelativeSForSample(end_s);
  std::vector<mapping::ElementId> lane_ids;
  lane_ids.reserve(index1 - index0 + 1);
  for (int i = index0; i <= index1; ++i) {
    lane_ids.push_back(lane_path().lane_ids()[i]);
  }
  return {semantic_map_manager_, std::move(lane_ids),
          rel_s0 / smooth_lanes_[index0].s_max(),
          rel_s1 / smooth_lanes_[index1].s_max()};
}

void SmoothLanePath::SendCanvas() const {
  std::vector<Vec3d> lane_points;
  const auto &coordinate_converter =
      semantic_map_manager_->coordinate_converter();
  for (int i = 0; i < lanes_.size(); ++i) {
    for (int j = 0; j < lanes_[i]->polyline().points_size(); ++j) {
      const auto &point = lanes_[i]->polyline().points(j);
      const Vec3d global(point.longitude(), point.latitude(), point.altitude());
      lane_points.push_back(coordinate_converter.GlobalToSmooth(global));
    }
  }

  vis::Canvas *canvas = &(vantage_client_man::GetCanvas("planner/route"));
  canvas->SetGroundZero(1);
  canvas->DrawLineStrip(lane_points, vis::Color(0.7, 0.5, 0.5), 3);
  vantage_client_man::FlushAll();
}

}  // namespace planner
}  // namespace qcraft
