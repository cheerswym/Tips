#ifndef ONBOARD_PERCEPTION_RETROREFLECTOR_DETECTOR_H
#define ONBOARD_PERCEPTION_RETROREFLECTOR_DETECTOR_H

#include <limits>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/str_format.h"
#include "onboard/async/thread_pool.h"
#include "onboard/maps/local_imagery.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/vec.h"
#include "onboard/perception/range_image/range_image.h"
#include "onboard/perception/retroreflector.h"
#include "opencv2/core.hpp"

namespace qcraft {

class RetroreflectorDetector {
 public:
  explicit RetroreflectorDetector(ThreadPool* thread_pool)
      : thread_pool_(thread_pool) {}

  Retroreflectors Detect(
      const absl::flat_hash_map<LidarId, RangeImage>& range_images,
      const LocalImagery& local_imagery,
      const CoordinateConverter& coordinate_converter, const VehiclePose& pose,
      const VehiclePose& pose_correction_result);

 private:
  ThreadPool* const thread_pool_ = nullptr;
};

}  // namespace qcraft

#endif  // ONBOARD_PERCEPTION_RETROREFLECTOR_DETECTOR_H
