#include "onboard/perception/tracker/track_classifier/label_convert_utils.h"

#include "onboard/lite/logging.h"

namespace qcraft::tracker {
using Label = labeling::Label;
using Zone = labeling::Zone;

track_classifier_label::Category ConvertToTcnLabel(Label::Category category) {
  switch (category) {
    case Label::CAR:
    case Label::CAR_DOOR:
      return track_classifier_label::VEH_CAR;
    case Label::TRAILER:
      return track_classifier_label::VEH_CONSTRUCTION;
    case Label::PED:
      return track_classifier_label::PED_ADULT;
    case Label::CHILD:
      return track_classifier_label::PED_CHILD;
    case Label::BICYCLIST:
      return track_classifier_label::XCY_BICYCLIST;
    case Label::MOTORCYCLIST:
      return track_classifier_label::XCY_MOTORCYCLIST;
    case Label::TRICYCLIST:
      return track_classifier_label::XCY_TRICYCLIST;
    case Label::ANIMAL:
      return track_classifier_label::OTH_ANIMAL;
    case Label::CONE:
      return track_classifier_label::STA_CONE;
    default:
      QLOG(FATAL) << "Should not reach here with label category " << category;
  }
}
track_classifier_label::Category ConvertToTcnLabel(Zone::ZoneType zone_type) {
  switch (zone_type) {
    case Zone::MIST:
      return track_classifier_label::OTH_MIST;
    case Zone::STATIC_OBJECT:
    case Zone::CURB:
      return track_classifier_label::STA_GENERAL;
    case Zone::BARRIER:
      return track_classifier_label::STA_BARRIER;
    case Zone::BLACK:
      return track_classifier_label::OTH_BLACK;
    case Zone::VEGETATION:
    case Zone::GREEN_BELT:
      return track_classifier_label::STA_VEGETATION;
    default:
      return track_classifier_label::OTH_UNKNOWN;
  }
}
track_classifier_label::Category ConvertToTcnLabel(MeasurementType type) {
  switch (type) {
    case MT_STATIC_OBJECT:
    case MT_ROAD:
      return track_classifier_label::STA_GENERAL;
    case MT_VEHICLE:
      return track_classifier_label::VEH_CAR;
    case MT_MOTORCYCLIST:
      return track_classifier_label::XCY_MOTORCYCLIST;
    case MT_PEDESTRIAN:
      return track_classifier_label::PED_ADULT;
    case MT_CYCLIST:
      return track_classifier_label::XCY_BICYCLIST;
    case MT_FOD:
      return track_classifier_label::OTH_FOD;
    case MT_UNKNOWN:
      return track_classifier_label::OTH_UNKNOWN;
    case MT_VEGETATION:
      return track_classifier_label::STA_VEGETATION;
    case MT_BARRIER:
      return track_classifier_label::STA_BARRIER;
    case MT_CONE:
      return track_classifier_label::STA_CONE;
    case MT_WARNING_TRIANGLE:
      return track_classifier_label::STA_WARNING_TRIANGLE;
    case MT_MIST:
      return track_classifier_label::OTH_MIST;
    case MT_FLYING_BIRD:
      return track_classifier_label::OTH_ANIMAL;
    default:
      QLOG(FATAL) << "Should not reach here with MeasurementType " << type;
  }
}
MeasurementType TcnTypeToMeasurementType(tcn_model::TcnType tcn_type) {
  switch (tcn_type) {
    case tcn_model::TcnType::TCN_VEHICLE:
      return MT_VEHICLE;
    case tcn_model::TcnType::TCN_PEDESTRIAN:
      return MT_PEDESTRIAN;
    case tcn_model::TcnType::TCN_CYCLIST:
      return MT_CYCLIST;
    case tcn_model::TcnType::TCN_VEGETATION:
      return MT_VEGETATION;
    case tcn_model::TcnType::TCN_BARRIER:
      return MT_BARRIER;
    case tcn_model::TcnType::TCN_UNKNOWN:
      return MT_UNKNOWN;
  }
}

}  // namespace qcraft::tracker
