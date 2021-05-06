#ifndef ONBOARD_MATH_GEOMETRY_LINE_INTERSECTION_H_
#define ONBOARD_MATH_GEOMETRY_LINE_INTERSECTION_H_

#include "onboard/math/geometry/polyline2d.h"
#include "onboard/math/vec.h"

namespace qcraft {

// The basic routines for finding intersection between two 2D lines. It is more
// efficient than similar functions in Segment2d such as GetIntersect() because
// the functions here don't require constructing Segment2d objects, which
// performs operations such as vector normalization that are unnecessary for
// many use cases of line intersection.

// The lines are given by a point and a tangent. The intersection point x can be
// computed by `p0 + v0 * s0 = p1 + v1 * s1`.
Vec2d FindIntersectionBetweenLinesWithTangents(Vec2d p0, Vec2d t0, Vec2d p1,
                                               Vec2d t1, double *s0,
                                               double *s1);

// Find intersections between two curves interpreted by discrete point. Return
// false if no intersection was found otherwise return the first intersection
// on curve_1
bool FindFirstIntersectionBetweenCurves(const Polyline2d &curve1,
                                        const Polyline2d &curve2,
                                        Vec2d *inter_point, double *arc_len1,
                                        double *arc_len2);

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_LINE_INTERSECTION_H_
