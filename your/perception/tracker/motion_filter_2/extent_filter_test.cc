
#include "onboard/perception/tracker/motion_filter_2/extent_filter.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
#include "onboard/perception/tracker/motion_filter_2/extent_model.h"
#include "onboard/perception/tracker/motion_filter_2/meas_model.h"
#include "onboard/perception/tracker/motion_filter_2/tracker_common.h"
#include "onboard/utils/file_util.h"

namespace qcraft::tracker {

TEST(ExtentFilter, FunctionTestAll) {
  BBoxState state;
  state << 1.0, 1.0, 1.0;

  MotionFilterParamProto params;
  QCHECK(file_util::TextFileToProto(
      "onboard/perception/tracker/motion_filter_2/data/car_model_car.pb.txt",
      &params));

  ExtentFilter<BBoxModel> ef(params);

  ef.Init(state);

  auto s = ef.state();
  EXPECT_DOUBLE_EQ(s.length(), 1.0);
  EXPECT_DOUBLE_EQ(s.width(), 1.0);
  EXPECT_DOUBLE_EQ(s.height(), 1.0);

  const double dt = 0.1;
  auto state_data = ef.ComputePrediction(ef.state(), ef.state_covariance(), dt);
  ef.Predict(dt);
  EXPECT_TRUE(ef.state().isApprox(state_data.state()));
  EXPECT_TRUE(ef.state_covariance().isApprox(state_data.state_cov()));
  EXPECT_DOUBLE_EQ(s.length(), 1.0);
  EXPECT_DOUBLE_EQ(s.width(), 1.0);
  EXPECT_DOUBLE_EQ(s.height(), 1.0);

  BBoxMeasurement bbox(1.1, 1.1, 0.0);
  ef.Update(bbox);
  QLOG(INFO) << "S_update: \n" << ef.state();
  QLOG(INFO) << "P_update: \n" << ef.state_covariance();
}

}  // namespace qcraft::tracker
