#ifndef ONBOARD_PERCEPTION_RETROREFLECTOR_H_
#define ONBOARD_PERCEPTION_RETROREFLECTOR_H_

#include <string>
#include <vector>

#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/proto/lidar.pb.h"
#include "opencv2/core.hpp"

namespace qcraft {

struct RoadSign {
  std::vector<Vec3d> points;
  Vec3d normal;
  Vec3d centroid;      // 3d
  Polygon2d contour;   // bird‘s eye view
  Box2d bounding_box;  // bird‘s eye view
  double min_z = 0.0;
  double max_z = 0.0;
  double area = 0.0;
  bool is_overhanging = false;

  std::string DebugString() const;
};
using RoadSigns = std::vector<RoadSign>;

struct RoadCylinder {
  std::vector<Vec3d> points;
  Vec3d axis;
  Vec3d centroid;      // 3d
  Polygon2d contour;   // bird‘s eye view
  Box2d bounding_box;  // bird‘s eye view
  double min_z = 0.0;
  double max_z = 0.0;
  bool is_overhanging = false;

  std::string DebugString() const;
};
using RoadCylinders = std::vector<RoadCylinder>;

struct Blob {
  std::vector<cv::Point> pixels;
  double area = 0.0;
  double circularity = 0.0;
  double inertia_ratio = 0.0;
  double convexity = 0.0;
  cv::Rect bounding_box;
  bool is_overhanging = false;

  std::string DebugString() const;
};
using Blobs = std::vector<Blob>;

struct Retroreflector {
  std::vector<Vec3d> points;
  // TODO(dong): Temporary save lidar_id for each retroreflector. This should be
  // removed after retroreflector merge and track is done.
  LidarId lidar_id = LDR_UNKNOWN;
  bool is_overhanging = false;
};
using Retroreflectors = std::vector<Retroreflector>;

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_RETROREFLECTOR_H_
