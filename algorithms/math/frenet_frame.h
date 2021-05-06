#ifndef ONBOARD_MATH_FRENET_FRAME_H_
#define ONBOARD_MATH_FRENET_FRAME_H_

#include <tuple>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "onboard/math/geometry/polygon2d.h"

namespace qcraft {

// Build a frenet frame based on discrete 2D points.
// Point in cartesian coordinate can be projected to frenet coordinate by XYToSL

class FrenetFrame {
 public:
  explicit FrenetFrame(const std::vector<Vec2d> &points) { Build(points); }

  double start_s() const { return s_knots_.front(); }
  double end_s() const { return s_knots_.back(); }
  double length() const { return s_knots_.back() - s_knots_.front(); }
  Vec2d InterpolateTangentByS(double s) const;
  Vec2d InterpolateTangentByXY(const Vec2d &xy) const;

  Vec2d SLToXY(const Vec2d &sl) const;
  // Transform a cartesian point to a frenet point with different output
  // verbosity
  Vec2d XYToSL(const Vec2d &xy) const;
  absl::StatusOr<Vec2d> XYToSLWithHeadingDiffLimit(
      const Vec2d &xy, double heading, double max_heading_diff) const;
  void XYToSL(const Vec2d &xy, Vec2d *sl, Vec2d *normal) const;
  void XYToSL(const Vec2d &xy, Vec2d *sl, Vec2d *normal, int *index,
              double *alpha) const;
  void XYToSL(const Vec2d &xy, Vec2d *sl, Vec2d *normal,
              std::pair<int, int> *raw_index_pair, double *alpha) const;
  // Transform a sequence of cartesian points to frenet points with different
  // output verbosity. The caller should guarantee that the cartesian points
  // go along the s direction so an O(N) algorithm can be implemented.
  std::vector<Vec2d> XYToSL(absl::Span<const Vec2d> xy) const;
  void XYToSL(absl::Span<const Vec2d> xy, absl::Span<Vec2d> sl,
              absl::Span<Vec2d> normal) const;
  void XYToSL(absl::Span<const Vec2d> xy, absl::Span<Vec2d> sl,
              absl::Span<Vec2d> normal,
              absl::Span<std::pair<int, int>> index_pairs,
              absl::Span<double> alpha) const;

  Vec2d FindAABBNearestPoint(const Polygon2d &polygon,
                             bool get_max_s = false) const;

 private:
  void Build(const std::vector<Vec2d> &points);

  std::tuple<Vec2d, Vec2d, double> GetInterpolationRange(double s) const;

  std::vector<Vec2d> points_;
  std::vector<double> s_knots_;
  // Inverse of n-1 segment length.
  std::vector<double> segment_len_inv_;
  std::vector<Vec2d> tangents_;

  std::vector<int> raw_indices_;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_FRENET_FRAME_H_
