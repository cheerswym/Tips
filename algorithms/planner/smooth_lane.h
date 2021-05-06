#ifndef ONBOARD_PLANNER_SMOOTH_LANE_H_
#define ONBOARD_PLANNER_SMOOTH_LANE_H_

#include <vector>

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

class SmoothLane {
 public:
  static constexpr double kLaneSampleInterval = 4.0;         // m.
  static constexpr double kLaneInterpolationInterval = 0.5;  // m.

  static constexpr int kLaneInterpolationDegree = 3;  // Cubic spline.

  static const std::function<double(double)> kNoLateralOffset;

  explicit SmoothLane(
      const SemanticMapManager &semantic_map_manager,
      mapping::ElementId lane_id,
      const std::function<double(double)> &lateral_offset = kNoLateralOffset);
  explicit SmoothLane(
      const SemanticMapManager &semantic_map_manager,
      const mapping::LaneProto &lane_proto,
      const std::function<double(double)> &lateral_offset = kNoLateralOffset);
  // Function lateral_offset gives the lateral offset (positive for left) at any
  // given arclength along the lane path.
  // Note: out-of-range arclength could be queried, due to the cross-lane
  // smoothness requirement (e.g. if a lane is 10m long, this constructor still
  // may ask for lateral_offset(-1.0) or lateral_offset(11.0)).
  // Note: Lateral offset should be small for the arclength error for the
  // resulting smooth curve to be small.
  explicit SmoothLane(
      const SemanticMapManager &semantic_map_manager,
      const mapping::LaneProto &lane_proto, const mapping::LaneProto *prev_lane,
      const mapping::LaneProto *next_lane, double sampling_start_offset,
      const std::function<double(double)> &lateral_offset = kNoLateralOffset);

  mapping::ElementId lane_id() const { return lane_id_; }

  const std::vector<double> &lane_point_s() const { return lane_point_s_; }
  const std::vector<Vec3d> &lane_points() const { return lane_points_; }

  const std::vector<Vec3d> &sampled_points() const { return sampled_points_; }

  const std::vector<double> &smooth_s() const { return smooth_s_; }
  const std::vector<Vec3d> &smooth_points() const { return smooth_points_; }

  // This is also equal to the lane's total length.
  double s_max() const { return s_max_; }
  double last_sample_s() const { return last_sample_s_; }

  Vec3d Sample(double s) const;
  std::vector<Vec3d> Sample(const std::vector<double> &s) const;

  Vec3d SampleTangent(double s) const;

  Vec3d SampleCurvatureNormal(double s) const;

  void SendCanvas() const;

 protected:
  void Build(const SemanticMapManager &semantic_map_manager,
             const mapping::LaneProto &lane,
             const mapping::LaneProto *prev_lane,
             const mapping::LaneProto *next_lane, double sampling_start_offset,
             const std::function<double(double)> &lateral_offset);

 private:
  const mapping::ElementId lane_id_;

  // Points from semantic map. S is the arclength of the corresonding point.
  std::vector<double> lane_point_s_;
  std::vector<Vec3d> lane_points_;

  // Knots and points for B-spline interpolation.
  // Sampled_points_ are sampled from the lane at uniform distance
  // kLaneSampleInterval via linear interpolation, with
  // (kLaneInterpolationDegree - 1) / 2 points padded on both front and back to
  // make the B-spline interpolation speed uniform near the ends.
  // Knots_.size() is equal to sampled_points_.size() + kLaneInterpolationDegree
  // with extraplated values padded on the back to support B-spline basis
  // computation.
  // With these padding, B-spline interpolation with s from 0 to s_max_ (the max
  // s of sampled_points_ before any padding) is supported.
  // s0 is an internal offset on top of s (s + s0 is between s0 and s0 + s_max_)
  // to make it usable in B-spline computation. The user should not be aware of
  // it.
  std::vector<double> knots_;
  std::vector<Vec3d> sampled_points_;
  double s0_ = 0.0;
  double s_max_ = 0.0;
  double last_sample_s_ = 0.0;

  // A canonical B-spline interpolation with uniform interval
  // kLaneInterpolationInterval. Sample() can be used to sample a point with s
  // anywhere between 0 and s_max, but if a uniform and fine sampling is needed,
  // no need to call Sample() which does the B-spline computation online; simply
  // use the smooth_points(). The result of Sample() is guaranteed to be
  // consistent with the smooth_points() and their smooth_s().
  std::vector<double> smooth_s_;
  std::vector<Vec3d> smooth_points_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SMOOTH_LANE_H_
