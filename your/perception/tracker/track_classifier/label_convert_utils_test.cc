#include "onboard/perception/tracker/track_classifier/label_convert_utils.h"

#include <iostream>
#include <limits>

#include "gtest/gtest.h"

namespace qcraft::tracker {
using Label = labeling::Label;
using Zone = labeling::Zone;
TEST(LabelConvertUtils, Category) {
  EXPECT_EQ(ConvertToTcnLabel(Label::CAR), track_classifier_label::VEH_CAR);
  EXPECT_EQ(ConvertToTcnLabel(Label::CAR_DOOR),
            track_classifier_label::VEH_CAR);
  EXPECT_EQ(ConvertToTcnLabel(Label::TRAILER),
            track_classifier_label::VEH_CONSTRUCTION);
  EXPECT_EQ(ConvertToTcnLabel(Label::PED), track_classifier_label::PED_ADULT);
  EXPECT_EQ(ConvertToTcnLabel(Label::CHILD), track_classifier_label::PED_CHILD);
  EXPECT_EQ(ConvertToTcnLabel(Label::BICYCLIST),
            track_classifier_label::XCY_BICYCLIST);
  EXPECT_EQ(ConvertToTcnLabel(Label::MOTORCYCLIST),
            track_classifier_label::XCY_MOTORCYCLIST);
  EXPECT_EQ(ConvertToTcnLabel(Label::TRICYCLIST),
            track_classifier_label::XCY_TRICYCLIST);
  EXPECT_EQ(ConvertToTcnLabel(Label::ANIMAL),
            track_classifier_label::OTH_ANIMAL);
  EXPECT_EQ(ConvertToTcnLabel(Label::CONE), track_classifier_label::STA_CONE);
}
TEST(LabelConvertUtils, Zone) {
  EXPECT_EQ(ConvertToTcnLabel(Zone::MIST), track_classifier_label::OTH_MIST);
  EXPECT_EQ(ConvertToTcnLabel(Zone::STATIC_OBJECT),
            track_classifier_label::STA_GENERAL);
  EXPECT_EQ(ConvertToTcnLabel(Zone::CURB), track_classifier_label::STA_GENERAL);
  EXPECT_EQ(ConvertToTcnLabel(Zone::BARRIER),
            track_classifier_label::STA_BARRIER);
  EXPECT_EQ(ConvertToTcnLabel(Zone::BLACK), track_classifier_label::OTH_BLACK);
  EXPECT_EQ(ConvertToTcnLabel(Zone::VEGETATION),
            track_classifier_label::STA_VEGETATION);
  EXPECT_EQ(ConvertToTcnLabel(Zone::GREEN_BELT),
            track_classifier_label::STA_VEGETATION);
}
TEST(LabelConvertUtils, Measurement) {
  EXPECT_EQ(ConvertToTcnLabel(MT_STATIC_OBJECT),
            track_classifier_label::STA_GENERAL);
  EXPECT_EQ(ConvertToTcnLabel(MT_ROAD), track_classifier_label::STA_GENERAL);
  EXPECT_EQ(ConvertToTcnLabel(MT_VEHICLE), track_classifier_label::VEH_CAR);
  EXPECT_EQ(ConvertToTcnLabel(MT_MOTORCYCLIST),
            track_classifier_label::XCY_MOTORCYCLIST);
}

TEST(LabelConvertUtils, TcnToMeasurement) {
  EXPECT_EQ(TcnTypeToMeasurementType(tcn_model::TcnType::TCN_VEHICLE),
            MT_VEHICLE);
  EXPECT_EQ(TcnTypeToMeasurementType(tcn_model::TcnType::TCN_PEDESTRIAN),
            MT_PEDESTRIAN);
  EXPECT_EQ(TcnTypeToMeasurementType(tcn_model::TcnType::TCN_CYCLIST),
            MT_CYCLIST);
  EXPECT_EQ(TcnTypeToMeasurementType(tcn_model::TcnType::TCN_VEGETATION),
            MT_VEGETATION);
  EXPECT_EQ(TcnTypeToMeasurementType(tcn_model::TcnType::TCN_BARRIER),
            MT_BARRIER);
  EXPECT_EQ(TcnTypeToMeasurementType(tcn_model::TcnType::TCN_UNKNOWN),
            MT_UNKNOWN);
}
}  // namespace qcraft::tracker
