#ifndef ONBOARD_CONTROL_CALIBRATION_TOOLS_CALIBRATION_TOOLS_H_
#define ONBOARD_CONTROL_CALIBRATION_TOOLS_CALIBRATION_TOOLS_H_

#include <memory>
#include <string>

#include "onboard/control/calibration/calibration_tools/calibration_interface.h"
#include "onboard/math/filters/mean_filter.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"

namespace qcraft {
namespace control {
namespace calibration {

class CalibrationTools {
 public:
  CalibrationTools(const std::string &dirname, const std::string &filename,
                   const std::string &method, double throttle_deadzone,
                   double brake_deadzone, bool is_save_sampledata);
  bool Process(double speed, double acceleration, double pitch_angle,
               double throttle, double brake, Chassis::GearPosition gear,
               vis::vantage::ChartDataProto *chart);
  void Reset();
  void SaveData();

 private:
  void PlotIdleProto(vis::vantage::ChartDataProto *chart);
  void PlotSlideProto(vis::vantage::ChartDataProto *chart);
  void PlotThrottleProto(vis::vantage::ChartDataProto *chart);
  void PlotBrakeProto(vis::vantage::ChartDataProto *chart);

  CalibrationProto AddReverseIdleProto(const CalibrationProto &proto);
  CalibrationProto AddReverseThrottleProto(const CalibrationProto &proto);
  CalibrationProto AddReverseBrakeProto(const CalibrationProto &proto);

  std::unique_ptr<apollo::common::MeanFilter> pitch_angle_filter_;
  std::unique_ptr<CalibrationInterface> calibration_method_;

  bool init_success_ = true;
  Method method_;

  CalibrationProto sample_proto_;
  CalibrationProto output_proto_;

  std::string filename_sample_;
  std::string filename_output_;

  bool is_save_sampledata_ = true;
};

}  // namespace calibration
}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CALIBRATION_TOOLS_CALIBRATION_TOOLS_H_
