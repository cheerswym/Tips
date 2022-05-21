#ifndef ONBOARD_UTILS_LABELS_VIEW_H_
#define ONBOARD_UTILS_LABELS_VIEW_H_

#include <deque>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "glog/logging.h"
#include "offboard/labeling/proto/label_frame.pb.h"
#include "onboard/lidar/vehicle_pose.h"
#include "onboard/lite/check.h"
#include "onboard/maps/imagery_manager.h"

namespace qcraft {

// Thread-safe
// Keep the view of Label proto。
class LabelsView {
 public:
  LabelsView();

  void UpdateLatestSpinTime(int64_t timestamp);

  void UpdateLabelFrame(const labeling::LabelFrameProto& proto);

  void ExportCurrentFrame(const CoordinateConverter& coordinate_converter,
                          labeling::LabelFrameProto* objects) const;

  void EnableShow(bool enable);

 private:
  mutable absl::Mutex mu_;

  bool enable_ GUARDED_BY(mu_);
  std::map<int64_t, labeling::LabelFrameProto> label_frame_proto_
      GUARDED_BY(mu_);
  int64_t cur_timestamp_ GUARDED_BY(mu_);
  std::unique_ptr<labeling::LabelFrameProto> cur_frame_ GUARDED_BY(mu_);
  mutable ImageryManager imagery_manager_;
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_LABELS_VIEW_H_
