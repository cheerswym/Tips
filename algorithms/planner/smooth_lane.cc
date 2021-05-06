#include "onboard/planner/smooth_lane.h"

#include <algorithm>
#include <memory>

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/util.h"
#include "onboard/utils/file_util.h"

DEFINE_bool(route_send_smoothing_canvas, false,
            "Send canvas for the lane smooth interpolation calculation.");

namespace qcraft {
namespace planner {

const std::function<double(double)> SmoothLane::kNoLateralOffset = [](double) {
  return 0.0;
};

namespace {

template <typename T>
T bspline_naive(int i, double t, const std::vector<double> &ts,
                const std::vector<T> &data, int degree) {
  const int n = ts.size();
  const auto ti = [&ts](int j) { return ts[j]; };
  std::vector<std::vector<double>> b(degree + 1, std::vector<double>(n, 0.0));
  b[0][i] = 1.0;
  for (int p = 1; p <= degree; ++p) {
    for (int j = 0; j < n; ++j) {
      if (j + p + 1 >= n || ti(j + p) == ti(j) || ti(j + p + 1) == ti(j + 1)) {
        b[p][j] = 0.0;
      } else {
        b[p][j] =
            (t - ti(j)) / (ti(j + p) - ti(j)) * b[p - 1][j] +
            (ti(j + p + 1) - t) / (ti(j + p + 1) - ti(j + 1)) * b[p - 1][j + 1];
      }
    }
  }
  T ret = T();
  for (int j = 0; j < data.size(); ++j) {
    ret += data[j] * b[degree][j];
  }
  return ret;
}

// De Boor's algorithm for evaluating B-spline.
// https://en.wikipedia.org/wiki/De_Boor%27s_algorithm
// TODO(Fang) return basis function derivatives for heading and curvature
// calculation.
template <typename T>
T bspline_deboor(int i, double t, const std::vector<double> &ts,
                 const std::vector<T> &data, int degree) {
  std::vector<T> d(degree + 1);
  for (int j = 0; j <= degree; ++j) {
    d[j] = data[i + j - degree];
  }

  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const double t0 = ts[i + j - degree];
      const double t1 = ts[i + j + 1 - r];
      const double alpha = (t - t0) / (t1 - t0);
      d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
    }
  }
  return d[degree];
}

template <typename T>
T bspline(int i, double t, const std::vector<double> &ts,
          const std::vector<T> &data, int degree) {
  return bspline_deboor(i, t, ts, data, degree);
}

struct ProcessedLane {
  std::vector<Vec3d> points;
  std::vector<double> s;

  ProcessedLane(const SemanticMapManager &semantic_map_manager,
                const mapping::LaneProto &lane) {
    double s = 0.0;
    const auto &coordinate_converter =
        semantic_map_manager.coordinate_converter();
    for (int j = 0; j < lane.polyline().points_size(); ++j) {
      const auto &point = lane.polyline().points(j);
      const Vec3d global(point.longitude(), point.latitude(), point.altitude());
      const Vec3d smooth = coordinate_converter.GlobalToSmooth(global);
      if (!points.empty()) {
        const double length = smooth.DistanceTo(points.back());
        s += length;
        DCHECK_GT(s, 0.001);  // Lane segments should well conditioned.
      }
      points.emplace_back(smooth);
      this->s.push_back(s);
    }
  }
};

Vec3d SampleLane(const ProcessedLane &lane, double s, Vec3d *tangent) {
  QCHECK_GE(lane.s.size(), 2);
  int index = 0;
  if (s < 0.0) {
    // QLOG(WARNING) << "s = " << s << " out of processed lane bound [0, "
    //               << lane.s.back() << "). Maybe lane is too short?";
    index = 0;
  } else if (s >= lane.s.back()) {
    // QLOG(WARNING) << "s = " << s << " out of processed lane bound [0, "
    //               << lane.s.back() << "). Maybe lane is too short?";
    index = lane.s.size() - 2;
  } else {
    index =
        std::upper_bound(lane.s.begin(), lane.s.end(), s) - lane.s.begin() - 1;
  }
  QCHECK_GE(index, 0);
  QCHECK_LT(index + 1, lane.s.size());
  if (tangent != nullptr) {
    *tangent = (lane.points[index + 1] - lane.points[index]).normalized();
  }
  const double s0 = lane.s[index];
  const double s1 = lane.s[index + 1];
  const double alpha = (s - s0) / (s1 - s0);
  return Lerp(lane.points[index], lane.points[index + 1], alpha);
}

}  // namespace

SmoothLane::SmoothLane(const SemanticMapManager &semantic_map_manager,
                       mapping::ElementId id,
                       const std::function<double(double)> &lateral_offset)
    : SmoothLane(semantic_map_manager,
                 semantic_map_manager.FindLaneByIdOrDie(id), lateral_offset) {}

SmoothLane::SmoothLane(const SemanticMapManager &semantic_map_manager,
                       const mapping::LaneProto &lane_proto,
                       const std::function<double(double)> &lateral_offset)
    : lane_id_(lane_proto.id()) {
  Build(semantic_map_manager, lane_proto, nullptr, nullptr, 0.0,
        lateral_offset);
}

SmoothLane::SmoothLane(const SemanticMapManager &semantic_map_manager,
                       const mapping::LaneProto &lane_proto,
                       const mapping::LaneProto *prev_lane,
                       const mapping::LaneProto *next_lane,
                       double sampling_start_offset,
                       const std::function<double(double)> &lateral_offset)
    : lane_id_(lane_proto.id()) {
  Build(semantic_map_manager, lane_proto, prev_lane, next_lane,
        sampling_start_offset, lateral_offset);
}

void SmoothLane::Build(const SemanticMapManager &semantic_map_manager,
                       const mapping::LaneProto &lane_proto,
                       const mapping::LaneProto *prev_lane,
                       const mapping::LaneProto *next_lane,
                       double sampling_start_offset,
                       const std::function<double(double)> &lateral_offset) {
  // Load and process the specified lane.
  const ProcessedLane processed_lane(semantic_map_manager, lane_proto);
  lane_points_ = processed_lane.points;
  lane_point_s_ = processed_lane.s;

  std::unique_ptr<ProcessedLane> processed_prev_lane;
  if (prev_lane != nullptr) {
    processed_prev_lane =
        std::make_unique<ProcessedLane>(semantic_map_manager, *prev_lane);
  }
  std::unique_ptr<ProcessedLane> processed_next_lane;
  if (next_lane != nullptr) {
    processed_next_lane =
        std::make_unique<ProcessedLane>(semantic_map_manager, *next_lane);
  }

  // We can't handle very short lanes yet due to smooth intepolation
  // requirements.
  // TODO(Fang) handle them.
  QCHECK_GE(lane_points_.size(), 2);
  QCHECK_GT(lane_point_s_.back(), kLaneSampleInterval)
      << lane_proto.DebugString();

  VLOG(2) << "Lane points:";
  for (int i = 0; i < lane_points_.size(); ++i) {
    VLOG(2) << i << ": s = " << lane_point_s_[i] << " ("
            << lane_points_[i].transpose() << ")";
  }

  // Uniformly sample the lane.
  int index = 0;
  double s = sampling_start_offset;
  last_sample_s_ = s;
  VLOG(2) << "Sampled points:";
  while (true) {
    while (index + 1 < lane_point_s_.size() && s >= lane_point_s_[index + 1]) {
      ++index;
    }
    if (index + 1 >= lane_point_s_.size()) break;
    const double alpha = (s - lane_point_s_[index]) /
                         (lane_point_s_[index + 1] - lane_point_s_[index]);
    const Vec3d point =
        Lerp(lane_points_[index], lane_points_[index + 1], alpha);
    const Vec3d normal(
        Vec2d(Vec3d(lane_points_[index + 1] - lane_points_[index]))
            .Perp()
            .normalized(),
        0.0);
    VLOG(2) << sampled_points_.size() << ": " << index << ", s = " << s
            << " alpha = " << alpha << ", [" << lane_point_s_[index] << ", "
            << lane_point_s_[index + 1] << "] -> (" << point.transpose()
            << ") normal = " << normal.transpose()
            << " lateral offset = " << lateral_offset(s) << " final sample: "
            << (point + normal * lateral_offset(s)).transpose();
    knots_.push_back(s);
    sampled_points_.push_back(point + normal * lateral_offset(s));
    last_sample_s_ = s;
    s += kLaneSampleInterval;
  }
  // Equivalent to SemanticMapManager::GetLaneLengthOrDie(lane_id).
  s_max_ = lane_point_s_.back();

  // Smoothly interpolate between the sampled points.
  // Pad the points and knots on the front by <degree> additional units. The
  // additional knots have extrapolated s values below zero. This ensures when
  // the interpolation starts at the original front, the speed is already
  // uniform like the rest of the curve. The additional points are either
  // sampled from the previous lane from the back, or if the previous lane is
  // not available, reflected from the front portion of the original points
  // about the first original point, so that s = interval * (degree + 1) / 2
  // evaluates to exactly the first original point, so the curve starts from
  // exactly the desired position.
  constexpr int kPadding = (kLaneInterpolationDegree - 1) / 2;
  const double front_knot = knots_.front();  // Should be 0.
  const Vec3d front_point = sampled_points_.front();
  std::vector<double> knots_header(kPadding + 1);
  std::vector<Vec3d> points_header(kPadding + 1);
  for (int j = 0; j < kPadding + 1; ++j) {
    const double s = front_knot - (j + 1) * kLaneSampleInterval;
    knots_header[kPadding - j] = s;
    Vec3d point;
    Vec3d tangent;
    if (prev_lane != nullptr) {
      point = SampleLane(*processed_prev_lane,
                         processed_prev_lane->s.back() + s, &tangent);
    } else {
      point =
          2 * lane_points_.front() - SampleLane(processed_lane, -s, &tangent);
    }
    const Vec3d normal(Vec2d(tangent).Perp(), 0.0);
    points_header[kPadding - j] = point + normal * lateral_offset(s);
  }
  knots_.insert(knots_.begin(), knots_header.begin(), knots_header.end());
  sampled_points_.insert(sampled_points_.begin(), points_header.begin(),
                         points_header.end());

  // Similarly pad on the back, by similarly extrapolating knots and reflecting
  // points. We need one more for the padding on this side because the lane's
  // real length is (less than one sample interval) longer than knots_.back() so
  // we may have interpolation s going above knots_.back().
  const double back_knot = knots_.back();
  const Vec3d back_point = sampled_points_.back();
  std::vector<double> knots_tail(kPadding + 1);
  std::vector<Vec3d> points_tail(kPadding + 1);
  for (int j = 0; j < kPadding + 1; ++j) {
    const double s = back_knot + (j + 1) * kLaneSampleInterval;
    knots_tail[j] = s;
    Vec3d point;
    Vec3d tangent;
    if (next_lane != nullptr) {
      point = SampleLane(*processed_next_lane, s - s_max_, &tangent);
    } else {
      point = 2 * lane_points_.back() -
              SampleLane(processed_lane, s_max_ - (s - s_max_), &tangent);
    }
    const Vec3d normal(Vec2d(tangent).Perp(), 0.0);
    points_tail[j] = point + normal * lateral_offset(s);
  }
  knots_.insert(knots_.end(), knots_tail.begin(), knots_tail.end());
  sampled_points_.insert(sampled_points_.end(), points_tail.begin(),
                         points_tail.end());

  // Pad the knots for the interpolation degree on the back, with extrapolation,
  // to make the speed uniform at the end. No need to pad points here as they
  // won't be used; they are only needed for spline basis computation.
  for (int j = 0; j < kLaneInterpolationDegree; ++j) {
    knots_.push_back(knots_.back() + kLaneSampleInterval);
  }

  VLOG(2) << "Knots:";
  for (int i = 0; i < knots_.size(); ++i) {
    VLOG(2) << i << ": " << knots_[i];
  }
  VLOG(2) << "Padded sampled points:";
  for (int i = 0; i < sampled_points_.size(); ++i) {
    VLOG(2) << i << ": " << sampled_points_[i].transpose() << std::endl;
  }

  // B-spline interpolation of the lane.
  // s goes from s0_ to s_max_ + s0_, where s0_ = interval * (degree + 1) / 2
  // which is such that when s = s0_, the interpolation evaluates to exactly the
  // original first point before the padding above. L is the original total
  // arclength.
  s0_ = kLaneSampleInterval * (kLaneInterpolationDegree + 1) / 2;
  index = 0;
  s = s0_;
  VLOG(2) << "Interpolated points:";
  while (true) {
    if (s > s_max_ + s0_) break;
    while (index + 1 < knots_.size() && s >= knots_[index + 1]) ++index;
    DCHECK_LT(index + 1, knots_.size());
    const Vec3d point =
        bspline(index, s, knots_, sampled_points_, kLaneInterpolationDegree);
    VLOG(2) << smooth_points_.size() << ": " << index << ", s = " << s << " ("
            << point.transpose() << ")";
    smooth_s_.push_back(s - s0_);
    smooth_points_.push_back(point);
    s += kLaneInterpolationInterval;
  }

  if (FLAGS_route_send_smoothing_canvas) {
    SendCanvas();
  }
}

Vec3d SmoothLane::Sample(double s) const {
  QCHECK(!knots_.empty());
  s = std::clamp(s, 0.0, s_max_);
  const double internal_s = s + s0_;
  int index = FloorToInt((internal_s - knots_.front()) / kLaneSampleInterval);
  // Just in case there's numerical error in the truncation step above.
  while (knots_[index] > internal_s) --index;
  while (knots_[index + 1] <= internal_s) ++index;
  return bspline(index, internal_s, knots_, sampled_points_,
                 kLaneInterpolationDegree);
}

std::vector<Vec3d> SmoothLane::Sample(const std::vector<double> &s) const {
  std::vector<Vec3d> points;
  points.reserve(s.size());
  for (const double s_value : s) points.push_back(Sample(s_value));
  return points;
}

Vec3d SmoothLane::SampleTangent(double s) const {
  // TODO(Fang) use analytical solution of the tangent instead of finite
  // differencing. Finite differencing here works okay because smooth lane
  // splines are guaranteed to progress by arclength so the normalization is
  // always well defind, and the curvature is bounded so the error is bounded.
  // But analytical solution is still more accurate.
  constexpr double kFiniteDifferenceOffset = 0.001;  // 1mm.
  QCHECK(!knots_.empty());
  s = std::clamp(s, 0.0, s_max_);
  const double s0 = std::max(s - kFiniteDifferenceOffset, 0.0);
  const double s1 = std::min(s + kFiniteDifferenceOffset, s_max_);
  const Vec3d x0 = Sample(s0);
  const Vec3d x1 = Sample(s1);
  return (x1 - x0).normalized();
}

Vec3d SmoothLane::SampleCurvatureNormal(double s) const {
  // TODO(Fang) use analytical solution of the tangent instead of finite
  // differencing. Finite differencing here works okay because smooth lane
  // splines are guaranteed to progress by arclength so the normalization is
  // always well defind, and the curvature is bounded so the error is bounded.
  // But analytical solution is still more accurate.
  constexpr double kFiniteDifferenceOffset = 0.001;  // 1mm.
  QCHECK(!knots_.empty());
  s = std::clamp(s, 0.0, s_max_);
  const double s0 = std::max(s - kFiniteDifferenceOffset, 0.0);
  const double s1 = std::min(s + kFiniteDifferenceOffset, s_max_);
  const Vec3d t0 = SampleTangent(s0);
  const Vec3d t1 = SampleTangent(s1);
  return (t1 - t0) / (s1 - s0);
}

void SmoothLane::SendCanvas() const {
  vis::Canvas *canvas = &(vantage_client_man::GetCanvas(
      absl::StrCat("planner/route_lanes/lane/", lane_id_)));
  canvas->SetGroundZero(1);
  for (int i = 0; i < lane_points_.size(); ++i) {
    canvas->DrawPoint(lane_points_[i], vis::Color(0.5, 0.25, 0.25), 2);
  }
  canvas->DrawLineStrip(lane_points_, vis::Color(0.4, 0.2, 0.2), 1);

  canvas = &(vantage_client_man::GetCanvas(
      absl::StrCat("planner/route_lanes/samples/", lane_id_)));
  canvas->SetGroundZero(1);
  for (int i = 0; i < sampled_points_.size(); ++i) {
    canvas->DrawPoint(sampled_points_[i], vis::Color(0.25, 0.5, 0.25), 2);
  }
  canvas->DrawLineStrip(sampled_points_, vis::Color(0.2, 0.4, 0.2), 1);

  canvas = &(vantage_client_man::GetCanvas(
      absl::StrCat("planner/route_lanes/smooth/", lane_id_)));
  canvas->SetGroundZero(1);
  for (int i = 0; i < smooth_points_.size(); ++i) {
    canvas->DrawPoint(smooth_points_[i], vis::Color(0.25, 0.25, 0.5), 2);
  }
  canvas->DrawLineStrip(smooth_points_, vis::Color(0.2, 0.2, 0.4), 1);

  vantage_client_man::FlushAll();
}

}  // namespace planner
}  // namespace qcraft
