#ifndef ONBOARD_MATH_GEOMETRY_HALFPLANE_H_
#define ONBOARD_MATH_GEOMETRY_HALFPLANE_H_

#include <string>
#include <vector>

#include "onboard/math/geometry/halfplane.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/vec.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft {

// A halfplane represents half of a 2D space defined by a start point and an end
// point. The direction is pointing from start to end, and the left side is
// considered as the "inside" of the halfplane.
class HalfPlane {
 public:
  HalfPlane() = default;

  explicit HalfPlane(const HalfPlaneProto &proto) { FromProto(proto); }

  HalfPlane(const Vec2d &start, const Vec2d &end, const Vec2d &dir)
      : start_(start), end_(end), center_((start + end) * 0.5), tangent_(dir) {}

  HalfPlane(const Vec2d &start, const Vec2d &end)
      : start_(start), end_(end), center_((start + end) * 0.5) {
    const Vec2d d = end - start;
    tangent_ = d.Unit();
  }

  Vec2d start() const { return start_; }
  Vec2d end() const { return end_; }
  Vec2d center() const { return center_; }
  Vec2d tangent() const { return tangent_; }
  double length() const { return tangent_.Dot(end_ - start_); }

  // Compute longitudinal projection on the halfplane boundary.
  double lon_proj(Vec2d p) const { return tangent_.Dot(p - start_); }

  // Compute lateral signed distance to the halfplane boundary.
  double lat_proj(Vec2d p) const { return tangent_.CrossProd(p - start_); }

  // Convert a point from the coordiante where the halfplane is defined to the
  // halfplane's coordinate, where start_ is the origin, and tangent_ is the
  // x-axis.
  Vec2d Transform(Vec2d p) const {
    const Vec2d diff = p - start_;
    return Vec2d(tangent_.Dot(diff), tangent_.CrossProd(diff));
  }

  // Convert a point from halfplane coordinate to global coordinate.
  Vec2d InvTransform(Vec2d p) const {
    const Vec2d neg_y_dir(tangent_.x(), -tangent_.y());
    return Vec2d(neg_y_dir.Dot(p), neg_y_dir.CrossProd(p)) + start_;
  }

  bool IsPointInside(Vec2d p) const {
    return tangent_.CrossProd(p - start_) > 0.0;
  }

  void FromProto(const HalfPlaneProto &proto) {
    *this =
        HalfPlane(Vec2dFromProto(proto.start()), Vec2dFromProto(proto.end()));
  }

  void ToProto(HalfPlaneProto *proto) const {
    proto->mutable_start()->set_x(start_.x());
    proto->mutable_start()->set_y(start_.y());
    proto->mutable_end()->set_x(end_.x());
    proto->mutable_end()->set_y(end_.y());
  }

  std::string DebugString() const {
    return absl::StrCat("HalfPlane({", start_.x(), ", ", start_.y(), "}", ",{",
                        end_.x(), ", ", end_.y(), "})");
  }

 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d center_;
  // Caches the unit direction vector of the halfplane.
  Vec2d tangent_;
};

}  // namespace qcraft

#endif  // ONBOARD_MATH_GEOMETRY_HALFPLANE_H_
