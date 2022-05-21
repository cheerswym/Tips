#include "onboard/utils/labels_view.h"

#include <algorithm>
#include <memory>

#include "gflags/gflags.h"
#include "offboard/simulation/label/label_frame_filter.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/utils/map_util.h"

DEFINE_bool(label_filter_with_detection_rule, false,
            "Render the label bounding box with respect of detection rule");

DEFINE_bool(label_filter_with_tracker_rule, false,
            "Render the label bounding box with respect of tracker rule");

DEFINE_bool(label_filter_with_offroad_rule, false,
            "Render the label bounding box with respect of offroad rule");

DEFINE_bool(label_filter_with_obstacle_detection_rule, false,
            "Render the label bounding box with respect of offroad rule");

namespace qcraft {
LabelsView::LabelsView() : enable_(true) {}

void LabelsView::UpdateLabelFrame(const labeling::LabelFrameProto& frame) {
  absl::MutexLock l(&mu_);
  const int64_t timestamp = frame.header().timestamp();
  if (label_frame_proto_.count(timestamp)) {
    return;
  }
  // keep the last 5 frame for future matching.
  if (label_frame_proto_.size() == 5) {
    auto old_begin_v = label_frame_proto_.begin()->first;
    label_frame_proto_[timestamp] = frame;
    if (timestamp < old_begin_v) {
      // if new item is inserted from the front, pop back
      label_frame_proto_.erase(prev(label_frame_proto_.end()));
    } else {
      label_frame_proto_.erase(label_frame_proto_.begin());
    }
  } else {
    label_frame_proto_[timestamp] = frame;
  }
}

void LabelsView::ExportCurrentFrame(
    const CoordinateConverter& coordinate_converter,
    labeling::LabelFrameProto* frame_out) const {
  absl::MutexLock l(&mu_);
  if (!enable_) {
    return;
  }

  if (cur_frame_) {
    if (FLAGS_label_filter_with_detection_rule) {
      FilterWithDetectionRange(cur_frame_.get());
    }
    if (FLAGS_label_filter_with_tracker_rule) {
      FilterWithTrackerRule(cur_frame_.get());
    }
    if (FLAGS_label_filter_with_offroad_rule) {
      FilterWithOffroadZone(coordinate_converter, &imagery_manager_,
                            cur_frame_.get());
    }
    if (FLAGS_label_filter_with_obstacle_detection_rule) {
      FilterWithObstacleDetectionRule(coordinate_converter, &imagery_manager_,
                                      cur_frame_.get());
    }
    *frame_out = *cur_frame_;
  }
}

void LabelsView::UpdateLatestSpinTime(int64_t timestamp) {
  absl::MutexLock l(&mu_);
  const auto* frame = FindOrNull(label_frame_proto_, timestamp);
  if (frame) {
    cur_frame_ = std::make_unique<labeling::LabelFrameProto>();
    *cur_frame_ = *frame;
    cur_timestamp_ = timestamp;
  } else {
    if (std::abs(timestamp - cur_timestamp_) > 100000) {
      VLOG(5) << "Lable Frame expired: " << timestamp
              << " cur:" << cur_timestamp_
              << " diff:" << timestamp - cur_timestamp_;
      cur_frame_.reset();
    }
  }
}

void LabelsView::EnableShow(bool enable) {
  absl::MutexLock l(&mu_);
  enable_ = enable;
}
}  // namespace qcraft
