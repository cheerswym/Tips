#include "onboard/perception/registration/registration_common_tool.h"

#include <random>
#include <utility>

namespace qcraft {
namespace reg_tool {
void DownSamplePointsUsingFixedStep(std::vector<Vec3d>* points,
                                    int max_num_points) {
  DCHECK_GT(points->size(), max_num_points);
  const float step = points->size() / static_cast<float>(max_num_points);
  int index = 0;
  float index_f = 0.0f;
  const int num_points = points->size();
  while (true) {
    const int i = RoundToInt(index_f);
    if (i >= num_points) break;
    (*points)[index++] = (*points)[i];
    index_f += step;
  }
  DCHECK_LE(index, num_points);
  points->resize(index);
}

void DownSamplePointsUsingFisherYatesShuffle(std::vector<Vec3d>* points,
                                             int max_num_points) {
  DCHECK_GT(points->size(), max_num_points);
  std::mt19937 e;
  std::uniform_int_distribution<int> u;

  size_t left = points->size();
  auto index = points->begin();
  while (max_num_points--) {
    auto r = index;
    std::advance(r, u(e) % left);
    std::swap(*index, *r);
    ++index;
    --left;
  }
  points->resize(std::distance(points->begin(), index));
}
}  // namespace reg_tool
}  // namespace qcraft
