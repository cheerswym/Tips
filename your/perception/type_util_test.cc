#include "onboard/perception/type_util.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/math/test_util.h"

namespace qcraft::type_util {

TEST(ToMeasurementTypeTest, TestVehicle) {
  EXPECT_EQ(ToMeasurementType(OT_VEHICLE), MT_VEHICLE);
}

TEST(ToMeasurementTypeTest, ShouldFail) {
  EXPECT_EQ(ToMeasurementType(static_cast<ObjectType>(100)), MT_UNKNOWN);
}

TEST(ToObjectTypeTest, TestUnknowns) {
  EXPECT_EQ(ToObjectType(MT_UNKNOWN), OT_UNKNOWN_MOVABLE);
  EXPECT_EQ(ToObjectType(MT_STATIC_OBJECT), OT_UNKNOWN_STATIC);
}

TEST(ToObjectTypeTest, ShouldFail) {
  EXPECT_DEATH(ToObjectType(MT_ROAD), "Should not");
}

TEST(ToObjectTypeTest, ShouldFail2) {
  EXPECT_DEATH(ToObjectType(MT_FLYING_BIRD), "Should not");
}

}  // namespace qcraft::type_util
