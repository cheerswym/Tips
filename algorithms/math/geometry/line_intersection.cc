#include "onboard/math/geometry/line_intersection.h"

#include <limits>

#include "onboard/lite/check.h"

namespace qcraft {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}
Vec2d FindIntersectionBetweenLinesWithTangents(Vec2d p0, Vec2d t0, Vec2d p1,
                                               Vec2d t1, double *s0,
                                               double *s1) {
  DCHECK(s0 != nullptr);
  DCHECK(s1 != nullptr);
  // We have the following equation:
  // p0 + t0 * s0 = p1 + t1 * s1;
  // Expand it by xy we get the following two equations:
  // p0.x() + t0.x() * s0 = p1.x() + t1.x() * s1;
  // p0.y() + t0.y() * s0 = p1.y() + t1.y() * s1;
  //
  // Reshape the equations:
  // t0.x() * s0 - t1.x() * s1 = p1.x() - p0.x()
  // t0.y() * s0 - t1.y() * s1 = p1.y() - p0.y()
  //
  // Let p0p1 = p1 - p0;
  // The solution is three cross products.
  // det = t1.x() * t0.y() - t1.y() * t0.x() = t1.CrossProd(t0);
  // s0 = 1.0 / det * (-t1.y(), t1.x()).Dot(p1 - p0)
  // s0 = -p0p1.CrossProd(t1) * 1.0 / det;

  // s1 = 1.0 / det * (-t0.y(), t0.x()).Dot(p1 - p0)
  // s1 = -p0p1.CrossProd(t0) * 1.0 / det;
  const double det = t1.CrossProd(t0);
  constexpr double kEpsilon = 1e-20;
  if (std::abs(det) < kEpsilon) {
    *s0 = kInf;
    *s1 = kInf;
    return Vec2d(kInf, kInf);
  }
  const double inv_det = -1.0 / det;
  const Vec2d p0p1 = p1 - p0;
  *s0 = inv_det * p0p1.CrossProd(t1);
  *s1 = inv_det * p0p1.CrossProd(t0);
  return p0 + *s0 * t0;
}

bool FindFirstIntersectionBetweenCurves(const Polyline2d &curve1,
                                        const Polyline2d &curve2,
                                        Vec2d *inter_point, double *arc_len1,
                                        double *arc_len2) {
  QCHECK_NOTNULL(inter_point);

  for (int i = 0; i + 1 < curve1.points().size(); ++i) {
    const Segment2d seg1(curve1.points()[i], curve1.points()[i + 1]);
    for (int j = 0; j + 1 < curve2.points().size(); ++j) {
      const Segment2d seg2(curve2.points()[j], curve2.points()[j + 1]);
      if (seg1.GetIntersect(seg2, inter_point)) {
        *arc_len1 =
            curve1.point_s()[i] + inter_point->DistanceTo(curve1.points()[i]);

        *arc_len2 =
            curve2.point_s()[j] + inter_point->DistanceTo(curve2.points()[j]);

        return true;
      }
    }
  }
  return false;
}
}  // namespace qcraft
