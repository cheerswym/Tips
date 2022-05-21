#ifndef ONBOARD_PERCEPTION_REGISTRATION_REGISTRATION_COMMON_TOOL_H_
#define ONBOARD_PERCEPTION_REGISTRATION_REGISTRATION_COMMON_TOOL_H_

#include <vector>

#include "onboard/math/vec.h"
#include "pcl/point_cloud.h"

namespace qcraft {
namespace reg_tool {

template <typename PointType>
void DownSamplePoints(typename pcl::PointCloud<PointType>::Ptr point_cloud,
                      int max_num_points) {
  DCHECK_GT(point_cloud->points.size(), max_num_points);
  const float step =
      point_cloud->points.size() / static_cast<float>(max_num_points);
  int index = 0;
  float index_f = 0.0f;
  const int num_points = point_cloud->points.size();
  while (true) {
    const int i = RoundToInt(index_f);
    if (i >= num_points) break;
    point_cloud->points[index++] = point_cloud->points[i];
    index_f += step;
  }
  DCHECK_LE(index, num_points);
  point_cloud->points.resize(index);
  point_cloud->height = 1;
  point_cloud->width = index;
}

void DownSamplePointsUsingFixedStep(std::vector<Vec3d>* points,
                                    int max_num_points);

void DownSamplePointsUsingFisherYatesShuffle(std::vector<Vec3d>* points,
                                             int max_num_points);

}  // namespace reg_tool
}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_REGISTRATION_REGISTRATION_COMMON_TOOL_H_
