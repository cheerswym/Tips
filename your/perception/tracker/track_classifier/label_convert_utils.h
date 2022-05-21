#ifndef ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LABEL_CONVERT_UTILS_H_
#define ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LABEL_CONVERT_UTILS_H_
#include "offboard/labeling/proto/label.pb.h"
#include "offboard/labeling/proto/track_classifier_label.pb.h"
#include "onboard/perception/tracker/track_classifier/proto/tcn_model.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::tracker {
track_classifier_label::Category ConvertToTcnLabel(
    labeling::Label::Category category);
track_classifier_label::Category ConvertToTcnLabel(
    labeling::Zone::ZoneType zone_type);
track_classifier_label::Category ConvertToTcnLabel(MeasurementType type);
MeasurementType TcnTypeToMeasurementType(tcn_model::TcnType tcn_type);
}  // namespace qcraft::tracker

#endif  // ONBOARD_PERCEPTION_TRACKER_TRACK_CLASSIFIER_LABEL_CONVERT_UTILS_H_
