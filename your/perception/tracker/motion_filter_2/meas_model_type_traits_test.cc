#include "onboard/perception/tracker/motion_filter_2/meas_model_type_traits.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(Meas_Model_to_State, StateType) {
  bool flag =
      std::is_same<typename StateTypeFromMeasType<PositionMeasurement>::type,
                   void>::value;
  QCHECK(flag);
  flag = std::is_same<typename StateTypeFromMeasType<SpeedMeasurement>::type,
                      void>::value;
  QCHECK(flag);
  flag = std::is_same<typename StateTypeFromMeasType<PosSpeedMeasurement>::type,
                      void>::value;
  QCHECK(flag);
  flag = std::is_same<typename StateTypeFromMeasType<HeadingMeasurement>::type,
                      CarState>::value;
  QCHECK(flag);
  flag = std::is_same<typename StateTypeFromMeasType<VelocityMeasurement>::type,
                      CarState>::value;
  QCHECK(flag);
}

}  // namespace qcraft::tracker
