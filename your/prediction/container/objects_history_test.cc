#include "onboard/prediction/container/objects_history.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace prediction {

TEST(ObjectsHistoryTest, Test_ObjectHistory_Push) {
  ObjectsHistory objects_history(3);
  CoordinateConverter loc_converter;
  EXPECT_EQ(objects_history["0"].empty(), true);
  EXPECT_EQ(objects_history["0"].Push(
                1.0, PredictionObject(ObjectProto(), loc_converter)),
            true);
  EXPECT_EQ(objects_history["0"].Push(
                1.0, PredictionObject(ObjectProto(), loc_converter)),
            false);
}

}  // namespace prediction
}  // namespace qcraft
