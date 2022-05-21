#include "onboard/perception/retroreflector.h"

#include <string>

#include "absl/strings/str_format.h"

namespace qcraft {

std::string RoadSign::DebugString() const {
  return absl::StrFormat(
      "normal: [%.2f, %.2f, %.2f], centroid: [%.2f, %.2f, %.2f], bbox_width: "
      "%.2f, bbox_length: %.2f, min_z: %.2f, max_z: %.2f, area: %.2f, "
      "is_overhanging? %s",
      normal.x(), normal.y(), normal.z(), centroid.x(), centroid.y(),
      centroid.z(), bounding_box.width(), bounding_box.length(), min_z, max_z,
      area, is_overhanging ? "YES" : "NO");
}

std::string RoadCylinder::DebugString() const {
  return absl::StrFormat(
      "axis: [%.2f, %.2f, %.2f], centroid: [%.2f, %.2f, %.2f], bbox_width: "
      "%.2f, bbox_length: %.2f, min_z: %.2f, max_z: %.2f, is_overhanging? %s",
      axis.x(), axis.y(), axis.z(), centroid.x(), centroid.y(), centroid.z(),
      bounding_box.width(), bounding_box.length(), min_z, max_z,
      is_overhanging ? "YES" : "NO");
}

std::string Blob::DebugString() const {
  return absl::StrFormat(
      "area: %.2f, circularity: %.2f, inertia_ratio: %.2f, convexity: %.2f, "
      "bounding_box(x,y,width,height): %d %d %d %d, is_overhanging? %s",
      area, circularity, inertia_ratio, convexity, bounding_box.x,
      bounding_box.y, bounding_box.width, bounding_box.height,
      is_overhanging ? "YES" : "NO");
}

}  // namespace qcraft
